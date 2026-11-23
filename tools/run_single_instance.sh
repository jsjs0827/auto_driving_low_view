#!/usr/bin/env bash
# Optional application launcher. It does not change the SDK's locking or APIs.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: run_single_instance.sh --resource NAME [--lock-dir DIR] -- PROGRAM [ARG ...]

Run one foreground application for a named resource. Every cooperating launcher
must use the same resource and lock directory. The default directory is
/tmp/perception-v2-single-instance-UID (private to the current user).
Resource names accept letters, digits, '.', '_' and '-' (1..128 characters).
A competing launch fails immediately with exit code 75. The lock is advisory:
programs started without this launcher are unaffected. No process is killed.
The child's exit code is preserved; a child may independently return 75 too.
flock directly execs the foreground program: it keeps the launcher's PID and
receives signals directly. The lock FD is inherited; surviving descendants can
retain the lock until their last copy closes. Keep the program in foreground.
Requires Bash, util-linux flock (--no-fork, --verbose, --conflict-exit-code), stat.
EOF
}

fail() { printf 'run_single_instance: %s\n' "$*" >&2; exit 2; }
resource=''
lock_dir="/tmp/perception-v2-single-instance-${UID}"
while (($#)); do
    case "$1" in
        --resource)
            (($# >= 2)) || fail '--resource requires a name'
            resource=$2; shift 2 ;;
        --lock-dir)
            (($# >= 2)) || fail '--lock-dir requires a directory'
            lock_dir=$2; shift 2 ;;
        --help|-h) usage; exit 0 ;;
        --) shift; break ;;
        *) fail "unknown option: $1 (use -- before the program)" ;;
    esac
done
[[ ${#resource} -ge 1 && ${#resource} -le 128 &&
   $resource != *[!a-zA-Z0-9._-]* && $resource != '.' && $resource != '..' ]] ||
    fail 'provide --resource using 1..128 letters/digits/dot/underscore/hyphen'
(($#)) || fail 'a foreground program is required after --'
[[ -n $lock_dir ]] || fail '--lock-dir must not be empty'
command -v flock >/dev/null 2>&1 || fail 'util-linux flock is required'
command -v stat >/dev/null 2>&1 || fail 'stat is required'

# Do not unlink a lock file after execution: removing a locked inode allows a
# competing process to lock a newly created file at the same pathname.
umask 077
[[ ! -L $lock_dir ]] || fail "lock directory is a symlink: $lock_dir"
mkdir -p -- "$lock_dir"
[[ -d $lock_dir && ! -L $lock_dir ]] || fail "invalid lock directory: $lock_dir"
[[ $(stat -c '%u' -- "$lock_dir") == "$UID" ]] || fail 'lock directory must belong to the current user'
mode=$(stat -c '%a' -- "$lock_dir")
(( (8#$mode & 0022) == 0 )) || fail 'lock directory must not be group/world writable'
lock_file="$lock_dir/$resource.lock"
[[ ! -L $lock_file ]] || fail "lock file is a symlink: $lock_file"
if [[ -e $lock_file ]]; then
    [[ -f $lock_file && $(stat -c '%u' -- "$lock_file") == "$UID" ]] ||
        fail 'existing lock file must be regular and owned by the current user'
fi
printf 'run_single_instance: resource=%s lock=%s; competing launches fail with code 75\n' \
       "$resource" "$lock_file" >&2

# No separate supervisor remains: Bash -> flock -> foreground program all exec
# in the same PID. Terminating that PID terminates the foreground program rather
# than releasing a supervisor's lock while leaving the program alive. The lock
# FD intentionally survives exec/fork; descendants retaining it delay release.
# Do not daemonize or close the lock FD. --verbose identifies lock contention;
# arguments are forwarded directly without eval or shell-string expansion.
exec flock --exclusive --nonblock --conflict-exit-code 75 --no-fork --verbose \
    -- "$lock_file" "$@"
