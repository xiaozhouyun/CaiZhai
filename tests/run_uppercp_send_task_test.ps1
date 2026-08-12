$ErrorActionPreference = 'Stop'

$testExe = Join-Path $env:TEMP 'uppercp_send_task_test.exe'

& gcc -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections `
    -I tests/uppercp_task_stubs -I Core/Inc `
    tests/uppercp_send_task_test.c Core/Src/UpperCP.c `
    '-Wl,--gc-sections' -lm -o $testExe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $testExe
exit $LASTEXITCODE
