#!/usr/bin/env python3
"""Inspect/extract selected ROS1 streams by HTTP range, without the dataset GUI.

Uses the pinned rosbags 0.11.3 reader format. Only indexes/chunks needed for the
requested interval are fetched. Normal ROS processing remains in the ROS env.
"""

import argparse
from collections import Counter, OrderedDict, defaultdict
import io
import json
import heapq
from pathlib import Path

import requests
from rosbags.interfaces import Connection
from rosbags.rosbag1 import Reader
from rosbags.rosbag1.reader import Header, RecordType, decompressors, read_uint32, read_bytes
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.convert.converter import generate_message_converter


class HTTPRangeFile(io.RawIOBase):
    def __init__(self, url, byte_limit=1_500_000_000):
        super().__init__()
        self.url = url
        self.session = requests.Session()
        response = self.session.head(url, timeout=45)
        response.raise_for_status()
        self.size = int(response.headers['Content-Length'])
        self.etag = response.headers.get('ETag')
        self.position = 0
        self.transferred = 0
        self.requests = 0
        self.byte_limit = byte_limit
        self.cache = OrderedDict()
        self.block_size = 65536

    def readable(self): return True
    def seekable(self): return True
    def tell(self): return self.position

    def seek(self, offset, whence=0):
        self.position = offset if whence == 0 else (
            self.position+offset if whence == 1 else self.size+offset)
        if self.position < 0:
            raise ValueError('Negative seek')
        return self.position

    def fetch(self, start, end):
        if self.transferred+end-start > self.byte_limit:
            raise RuntimeError('Configured network byte limit exceeded')
        response = self.session.get(self.url,
            headers={'Range': f'bytes={start}-{end-1}'}, timeout=90)
        response.raise_for_status()
        if response.status_code != 206 or len(response.content) != end-start:
            raise RuntimeError('Server did not honor the exact byte range')
        self.transferred += len(response.content)
        self.requests += 1
        return response.content

    def read(self, size=-1):
        if size < 0:
            raise ValueError('Unbounded remote reads are disabled')
        end = min(self.position+size, self.size)
        if end <= self.position:
            return b''
        if size >= self.block_size:
            value = self.fetch(self.position, end)
            self.position = end
            return value
        result = bytearray()
        while self.position < end:
            block = self.position//self.block_size
            start = block*self.block_size
            if block not in self.cache:
                self.cache[block] = self.fetch(start, min(start+self.block_size, self.size))
                while len(self.cache) > 128:
                    self.cache.popitem(last=False)
            self.cache.move_to_end(block)
            data = self.cache[block]
            count = min(end-self.position, len(data)-(self.position-start))
            result.extend(data[self.position-start:self.position-start+count])
            self.position += count
        return bytes(result)

    def readline(self, size=-1):
        result = bytearray()
        while size < 0 or len(result) < size:
            value = self.read(1)
            if not value: break
            result.extend(value)
            if value == b'\n': break
        return bytes(result)

    def close(self):
        if not self.closed:
            self.session.close()
        super().close()


class RemotePath:
    def __init__(self, stream): self.stream = stream
    def exists(self): return True
    def open(self, mode):
        if mode != 'rb': raise ValueError('Remote input is read-only')
        return self.stream
    def __str__(self): return self.stream.url


class IntervalReader(Reader):
    def open_metadata(self):
        self.bio = self.path.open('rb')
        if self.bio.readline() != b'#ROSBAG V2.0\n':
            raise ValueError('Not a ROS1 version 2 bag')
        header = Header.read(self.bio, RecordType.BAGHEADER)
        self.bio.seek(header.get_uint64('index_pos'))
        self.connections = [self.read_connection() for _ in range(header.get_uint32('conn_count'))]
        self.chunk_infos = [self.read_chunk_info() for _ in range(header.get_uint32('chunk_count'))]
        counts = Counter()
        for info in self.chunk_infos: counts.update(info.connection_counts)
        self.connections = [Connection(*c[0:5], counts[c.id], *c[6:]) for c in self.connections]

    def load_interval_indexes(self, topics, start, stop, bootstrap_topics=()):
        selected = {c.id for c in self.connections if c.topic in topics}
        bootstrap = {c.id for c in self.connections if c.topic in bootstrap_topics}
        bootstrap_chunks = set()
        for conn in bootstrap:
            match = next((info.pos for info in self.chunk_infos
                          if info.connection_counts.get(conn, 0)), None)
            if match is not None: bootstrap_chunks.add(match)
        indexes = defaultdict(list)
        chunks = [info for info in self.chunk_infos if info.pos in bootstrap_chunks
                  or (info.start_time < stop and info.end_time >= start
                      and selected.intersection(info.connection_counts))]
        for index, info in enumerate(chunks):
            self.bio.seek(info.pos)
            self.chunks[info.pos] = self.read_chunk()
            for _ in range(len(info.connection_counts)):
                self.read_index_data(info.pos, indexes)
            if index % 100 == 0:
                print(json.dumps({'indexed_chunks': index, 'selected_chunks':len(chunks),
                                  'downloaded_MB':round(self.path.stream.transferred/1e6,1)}),flush=True)
        self.indexes = {c.id:sorted(indexes[c.id]) for c in self.connections}

    def messages(self, connections=(), start=None, stop=None):
        """Read individual records directly from uncompressed remote chunks."""
        connections=list(connections) or self.connections
        connmap={c.id:c for c in self.connections}
        for entry in heapq.merge(*(self.indexes[c.id] for c in connections)):
            if start is not None and entry.time < start: continue
            if stop is not None and entry.time >= stop: break
            chunk=self.chunks[entry.chunk_pos]
            if chunk.decompressor is not decompressors['none']:
                raise ValueError('Selective remote extraction requires uncompressed chunks; '
                                 'use the standard rosbags reader for a local compressed bag')
            self.bio.seek(chunk.datapos+entry.offset)
            header=Header.read(self.bio)
            while header.get_uint8('op') == RecordType.CONNECTION:
                self.bio.seek(read_uint32(self.bio),1)
                header=Header.read(self.bio)
            if header.get_uint8('op') != RecordType.MSGDATA:
                raise ValueError('Expected message record at indexed offset')
            raw=read_bytes(self.bio,read_uint32(self.bio))
            yield connmap[header.get_uint32('conn')],entry.time,raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--offset', type=float, default=45.0)
    parser.add_argument('--seconds', type=float, default=20.0)
    args = parser.parse_args()
    stream = HTTPRangeFile(args.url)
    reader = IntervalReader(RemotePath(stream))
    try:
        reader.open_metadata()
        report = dict(url=args.url,source_bytes=stream.size,etag=stream.etag,
            start_ns=reader.start_time,stop_ns=reader.end_time,
            duration_seconds=reader.duration/1e9,chunk_count=len(reader.chunk_infos),
            topics=[dict(name=c.topic,type=c.msgtype,count=c.msgcount,
                         caller=c.ext.callerid,connection_id=c.id) for c in reader.connections],
            network_bytes=stream.transferred,network_requests=stream.requests)
        args.metadata.parent.mkdir(parents=True,exist_ok=True)
        args.metadata.write_text(json.dumps(report,indent=2)+'\n')
        if not args.output:
            print(json.dumps(report,indent=2),flush=True)
            return
        topics = {'/velodyne_1/velodyne_points', '/multisense/left/image_rect_color',
                  '/lidar_odometry'}
        start = reader.start_time+int(args.offset*1e9)
        stop = start+int(args.seconds*1e9)
        reader.load_interval_indexes(topics, start-int(1e9), stop+int(1e9),
                                     bootstrap_topics={'/tf_static'})
        source = get_typestore(Stores.ROS1_NOETIC)
        target = get_typestore(Stores.ROS2_HUMBLE)
        selected = [c for c in reader.connections if c.topic in topics | {'/tf_static'}]
        for c in selected:
            if c.msgtype not in source.types:
                source.register(get_types_from_msg(c.msgdef.data,c.msgtype))
            if c.msgtype not in target.types:
                definitions=get_types_from_msg(c.msgdef.data,c.msgtype)
                definitions.pop('std_msgs/msg/Header',None)
                target.register(definitions)
        converters = {c.id:generate_message_converter(source,target,c.msgtype,c.msgtype,{},
                          src_is2=False,dst_is2=True) for c in selected}
        counts=Counter()
        first_headers={}
        with Writer(args.output,version=8) as writer:
            conns={}
            for c in selected:
                if c.topic not in conns:
                    conns[c.topic]=writer.add_connection(c.topic,c.msgtype,typestore=target)
            tf_conn=writer.add_connection('/tf','tf2_msgs/msg/TFMessage',typestore=target)
            seen=set()
            for c,t,raw in reader.messages(connections=[c for c in selected if c.topic=='/tf_static']):
                if c.id in seen: continue
                seen.add(c.id)
                writer.write(conns[c.topic],start-int(1e9),converters[c.id](raw))
                value=source.deserialize_ros1(raw,c.msgtype)
                first_headers.setdefault('static_transforms',[]).extend([
                    dict(parent=tf.header.frame_id,child=tf.child_frame_id,
                         xyz=[tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z],
                         xyzw=[tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w])
                    for tf in value.transforms])
                counts[c.topic]+=1
            for c,t,raw in reader.messages(connections=[c for c in selected if c.topic in topics],
                                           start=start-int(1e9),stop=stop+int(1e9)):
                writer.write(conns[c.topic],t,converters[c.id](raw))
                counts[c.topic]+=1
                if c.topic=='/lidar_odometry':
                    value=source.deserialize_ros1(raw,c.msgtype)
                    types=target.types
                    stamp=value.header.stamp
                    header=types['std_msgs/msg/Header'](
                        stamp=types['builtin_interfaces/msg/Time'](sec=stamp.sec,nanosec=stamp.nanosec),
                        frame_id=value.header.frame_id)
                    p,q=value.pose.pose.position,value.pose.pose.orientation
                    transform=types['geometry_msgs/msg/TransformStamped'](header=header,
                        child_frame_id=value.child_frame_id,
                        transform=types['geometry_msgs/msg/Transform'](
                            translation=types['geometry_msgs/msg/Vector3'](p.x,p.y,p.z),
                            rotation=types['geometry_msgs/msg/Quaternion'](q.x,q.y,q.z,q.w)))
                    tf=types['tf2_msgs/msg/TFMessage'](transforms=[transform])
                    dispatch=stamp.sec*10**9+stamp.nanosec-1_000_000
                    writer.write(tf_conn,dispatch,target.serialize_cdr(tf,'tf2_msgs/msg/TFMessage'))
                    counts['/tf']+=1
                if c.topic not in first_headers:
                    value=source.deserialize_ros1(raw,c.msgtype)
                    first_headers[c.topic]=dict(frame=value.header.frame_id,
                        stamp_ns=value.header.stamp.sec*10**9+value.header.stamp.nanosec)
                    if hasattr(value,'width'):
                        first_headers[c.topic].update(width=value.width,height=value.height)
                    if hasattr(value,'child_frame_id'):
                        first_headers[c.topic].update(child=value.child_frame_id)
                if sum(counts.values())%100==0:
                    print(json.dumps({'messages':dict(counts),'downloaded_MB':round(stream.transferred/1e6,1)}),flush=True)
        report.update(network_bytes=stream.transferred,network_requests=stream.requests,
                      output=str(args.output),requested_start_ns=start,requested_stop_ns=stop,
                      selected_counts=dict(counts),first_headers=first_headers,
                      generated_tf_source='/lidar_odometry; original acquisition pose and stamp; dispatched at stamp minus 1 ms')
        args.metadata.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:report[k] for k in ['output','network_bytes','selected_counts','first_headers']},indent=2),flush=True)
    finally:
        reader.close()


if __name__ == '__main__': main()
