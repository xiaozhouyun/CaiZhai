$ErrorActionPreference = 'Stop'

$testExe = Join-Path $env:TEMP 'app_route_c_position_test.exe'

& gcc -std=c11 -Wall -Wextra -Werror -Wno-unused-function `
    -I tests/app_route_b_stubs -I Core/Inc `
    tests/app_route_c_position_test.c Core/Src/app.c -lm -o $testExe
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $testExe
exit $LASTEXITCODE
