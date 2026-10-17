$ErrorActionPreference = 'Stop'
$toolsRoot = Join-Path (Split-Path -Parent $PSScriptRoot) '.tools'
$armBin = Join-Path $toolsRoot 'arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi/bin'
$hostBin = Join-Path $toolsRoot 'w64devkit/bin'
foreach ($compiler in @((Join-Path $armBin 'arm-none-eabi-gcc.exe'), (Join-Path $hostBin 'make.exe'))) {
    if (-not (Test-Path -LiteralPath $compiler)) {
        throw "Required tool not found: $compiler"
    }
}
$env:PATH = "$armBin;$hostBin;C:\Program Files\Git\usr\bin;$env:PATH"
# Scope Git's ownership exceptions to this build process.
$env:GIT_CONFIG_COUNT = '2'
$env:GIT_CONFIG_KEY_0 = 'safe.directory'
$env:GIT_CONFIG_VALUE_0 = $PSScriptRoot.Replace('\', '/')
$env:GIT_CONFIG_KEY_1 = 'safe.directory'
$env:GIT_CONFIG_VALUE_1 = (Join-Path $PSScriptRoot 'src/config').Replace('\', '/')
# Match the user's Betaflight build-selection screenshot. CLOUD_BUILD selects
# an explicit feature set locally; it does not upload sources to a cloud service.
$buildOptions = @(
    'CLOUD_BUILD',
    'USE_SERIALRX', 'USE_SERIALRX_CRSF', 'USE_TELEMETRY', 'USE_TELEMETRY_CRSF',
    'USE_OSD', 'USE_OSD_SD', 'USE_DSHOT', 'USE_ACRO_TRAINER',
    'USE_GPS', 'USE_PINIO', 'USE_RANGEFINDER', 'USE_VTX',
    'USE_POSITION_HOLD', 'USE_ALTITUDE_HOLD', 'USE_MAG'
)
# Environment propagation avoids Windows recursive-make escaping of spaces.
$env:OPTIONS = $buildOptions -join ' '
$env:EXTRA_FLAGS = '-fno-merge-constants -fno-builtin-memcpy'
Push-Location $PSScriptRoot
try {
    # Preserve upstream LTO optimisation flags. Only add the Windows partition
    # workaround; overriding LTO_FLAGS would discard upstream link settings.
    # Rebuild the generated version object to force relinking: Make does not
    # track linker flags, and its recursive targets do not retain -W here.
    $versionObject = 'obj/main/STM32F405_JHEF405PRO/build/version.o'
    if (Test-Path -LiteralPath $versionObject) { Remove-Item -LiteralPath $versionObject }
    # The previous ELF merged FC_VERSION_STRING to an unaligned address while
    # its inlined copy used LDMIA (which faults on unaligned Cortex-M4 loads).
    # Keep constant merging disabled as a precaution; msp.c also explicitly
    # aligns the version array. Rebuild objects via the EXTRA_FLAGS hash.
    # Do not inline memcpy with assumed alignment of pooled string literals.
    # The library memcpy handles arbitrary source/destination byte alignment.
    & (Join-Path $hostBin 'make.exe') CONFIG=JHEF405PRO OSFAMILY=windows MINGW=1 CCACHE= CFLAGS_DISABLED=-fmerge-all-constants EXTRA_LD_FLAGS=@jhef405pro-link.rsp -j4
    if ($LASTEXITCODE -ne 0) { throw "Firmware build failed: $LASTEXITCODE" }
    & (Join-Path $PSScriptRoot 'check-msp-alignment.ps1') -NmPath (Join-Path $armBin 'arm-none-eabi-nm.exe')
    $firmwarePath = 'obj/betaflight_2026.6.2_STM32F405_JHEF405PRO.hex'
    $selectedPath = 'obj/betaflight_2026.6.2_JHEF405PRO_CRSF_ANALOG_POSHOLD_MAG_MSP_COPY_FIXED.hex'
    Copy-Item -LiteralPath $firmwarePath -Destination $selectedPath -Force
    Write-Host "Firmware: $(Join-Path $PSScriptRoot $selectedPath)"
} finally {
    Pop-Location
}
