"""Run host CMake/CTest with one spelling per Windows environment key."""
import os
import subprocess
import sys
from pathlib import Path
arguments = sys.argv[1:]
program = "cmake"
if arguments and arguments[0] == "ctest":
    program = arguments.pop(0)
environment = dict(os.environ)
if os.name == "nt":
    environment = {key.upper(): value for key, value in environment.items()}
    environment["VSLANG"]="1033"
    if "--msvc" in arguments:
        arguments.remove("--msvc")
        installation=Path(environment.get("VSINSTALLDIR",r"C:\Program Files\Microsoft Visual Studio\2022\Community"))
        vcvars=installation/"VC/Auxiliary/Build/vcvars64.bat"
        setup=subprocess.run(f'cmd.exe /d /s /c ""{vcvars}" >nul && set"',
                             env=environment,capture_output=True,text=True,encoding="mbcs")
        if setup.returncode:
            raise SystemExit(setup.stderr)
        for line in setup.stdout.splitlines():
            key,separator,value=line.partition("=")
            if separator and key:environment[key.upper()]=value
    environment["VSLANG"]="1033"
if "--log" in arguments:
    at=arguments.index("--log");log_path=Path(arguments[at+1]);del arguments[at:at+2]
    log_path.parent.mkdir(parents=True,exist_ok=True)
    process=subprocess.Popen([program,*arguments],env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    sys.stdout.reconfigure(encoding="utf-8",errors="replace")
    with log_path.open("w",encoding="utf-8") as output:
        for raw in process.stdout:
            try:line=raw.decode("utf-8")
            except UnicodeDecodeError:line=raw.decode("mbcs" if os.name=="nt" else "utf-8",errors="replace")
            output.write(line)
            if not line.lstrip().startswith(("Note: including file:","注意: 包含文件:")):
                print(line,end="",flush=True)
    raise SystemExit(process.wait())
raise SystemExit(subprocess.call([program, *arguments], env=environment))
