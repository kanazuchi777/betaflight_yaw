param(
    [string]$ElfPath = 'obj/main/betaflight_STM32F405_JHEF405PRO.elf',
    [string]$NmPath = '../.tools/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi/bin/arm-none-eabi-nm.exe'
)
$ErrorActionPreference = 'Stop'
$symbols = & $NmPath -S $ElfPath
if ($LASTEXITCODE -ne 0) { throw 'Cannot inspect firmware ELF symbols' }
$foundVersion = $false
foreach ($symbolLine in $symbols) {
    if ($symbolLine -match '^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+\w\s+mspFirmwareVersion(?:\.\d+)?$') {
        $foundVersion = $true
        $versionAddress = [Convert]::ToUInt32($Matches[1], 16)
        if (($versionAddress % 4) -ne 0) { throw "MSP version string is not word aligned: $symbolLine" }
        Write-Output "MSP firmware version alignment PASS: $symbolLine"
    }
}

# Check direct literal-address LDM copies in the MSP handshake handler too:
# aligning only the version left targetName/boardName/buildDate vulnerable.
$objdumpPath = Join-Path (Split-Path -Parent $NmPath) 'arm-none-eabi-objdump.exe'
$assembly = @(& $objdumpPath -d --disassemble=mspCommonProcessOutCommand.isra.0 $ElfPath)
if ($LASTEXITCODE -ne 0) { throw 'Cannot disassemble MSP handler' }
if (-not ($assembly -match '<mspCommonProcessOutCommand.isra.0>:')) {
    throw 'MSP handler symbol changed; update the machine-code alignment check'
}
$literals = @{}
foreach ($line in $assembly) {
    if ($line -match '^\s*([0-9a-f]+):.*\.word\s+0x([0-9a-f]+)') {
        $literals[$Matches[1]] = [Convert]::ToUInt32($Matches[2], 16)
    }
}
for ($lineIndex = 0; $lineIndex -lt $assembly.Count; $lineIndex++) {
    if ($assembly[$lineIndex] -notmatch '\bldmia(?:\.w)?\s+(r\d+)!?') { continue }
    $sourceRegister = $Matches[1]
    for ($previous = $lineIndex - 1; $previous -ge [Math]::Max(0, $lineIndex - 12); $previous--) {
        # Stop at the nearest instruction that computes/loads this register.
        if ($assembly[$previous] -notmatch "\b(?:ldr\w*|mov\w*|add\w*|sub\w*)(?:\.w)?\s+$sourceRegister,") { continue }
        if ($assembly[$previous] -match "\bldr(?:\.w)?\s+$sourceRegister,\s*\[pc,.*@\s*\(([0-9a-f]+)") {
            $literalAddress = $Matches[1]
            if (-not $literals.ContainsKey($literalAddress)) { throw 'Cannot resolve MSP literal address' }
            $copyAddress = $literals[$literalAddress]
            if (($copyAddress % 4) -ne 0) {
                throw ('Unaligned MSP LDM source 0x{0:X8}: {1}' -f $copyAddress, $assembly[$lineIndex])
            }
        }
        break
    }
}
Write-Output 'MSP direct literal-address LDM alignment PASS (not a full ARM memory-access audit)'
if (-not $foundVersion) { throw 'Missing explicitly aligned MSP firmware version storage' }
