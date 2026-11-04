# Save frames of the RViz window only. Runs on the Windows side, because WSLg
# windows drawn through the GPU read back black from inside Linux.
# Each line on standard input is a Windows file path: the window is saved there as
# a JPEG and "ok" is printed. An empty line or end of input stops the script.
param([int]$Quality = 95)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System; using System.Text; using System.Runtime.InteropServices; using System.Collections.Generic;
public class Rv {
  public delegate bool Enum(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] public static extern bool EnumWindows(Enum f, IntPtr l);
  [DllImport("user32.dll", CharSet=CharSet.Unicode)] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr h, IntPtr dc, uint f);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] public static extern bool EnableWindow(IntPtr h, bool on);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
  public static IntPtr Find() {
    IntPtr hit = IntPtr.Zero;
    EnumWindows((h, l) => { var s = new StringBuilder(512); GetWindowText(h, s, 512);
      if (IsWindowVisible(h) && s.ToString().Contains(" - RViz")) { hit = h; return false; } return true; }, IntPtr.Zero);
    return hit; }
}
'@
[void][Rv]::SetProcessDPIAware()
$h = [Rv]::Find()
if ($h -eq [IntPtr]::Zero) { throw 'No visible RViz window found' }
[void][Rv]::EnableWindow($h, $false)  # a stray mouse drag or scroll would move the RViz view mid-recording
$codec = [System.Drawing.Imaging.ImageCodecInfo]::GetImageEncoders() | Where-Object { $_.MimeType -eq 'image/jpeg' }
$opts = New-Object System.Drawing.Imaging.EncoderParameters 1
$opts.Param[0] = New-Object System.Drawing.Imaging.EncoderParameter ([System.Drawing.Imaging.Encoder]::Quality, [long]$Quality)
$r = New-Object Rv+RECT
[void][Rv]::GetWindowRect($h, [ref]$r)
$w = $r.R - $r.L; $ht = $r.B - $r.T
$bmp = New-Object System.Drawing.Bitmap $w, $ht
$g = [System.Drawing.Graphics]::FromImage($bmp)
while ($true) {
  $path = [Console]::In.ReadLine()
  if ([string]::IsNullOrEmpty($path)) { break }
  $dc = $g.GetHdc()
  [void][Rv]::PrintWindow($h, $dc, 2)
  $g.ReleaseHdc($dc)
  $bmp.Save($path, $codec, $opts)
  [Console]::Out.WriteLine('ok')
  [Console]::Out.Flush()
}
