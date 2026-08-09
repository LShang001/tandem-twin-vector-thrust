@echo off
rem Restore full PATH for ZCode agent shell (sandbox keeps only ZCode tools + npm).
rem Usage: call tools\shell-env.cmd && git status
rem NOTE: keep this file pure ASCII - cmd.exe parses .cmd with GBK codepage,
rem UTF-8 Chinese comments get mangled and break parsing.
rem Source of entries: HKLM Environment Path (machine), verified on this host.
set "PATH=C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0;C:\Windows\System32\OpenSSH;D:\Program Files\Git\cmd;D:\Program Files\Git\bin;D:\Program Files\nodejs;C:\Users\12631\AppData\Local\Programs\Python\Python312;C:\Users\12631\AppData\Local\Programs\Python\Python312\Scripts;C:\ProgramData\chocolatey\bin;C:\Program Files\PowerShell\7;C:\Program Files\GitHub CLI;C:\Users\12631\AppData\Roaming\npm;D:\Program Files\ZCode\resources\tools\ripgrep;D:\Program Files\ZCode\resources\tools\ugrep"
