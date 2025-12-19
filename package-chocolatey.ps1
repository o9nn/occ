# OpenCog Collection - Chocolatey Package Generator
# This script automates the creation of Chocolatey packages from build artifacts

param(
    [string]$ZipPath = "",
    [string]$Version = "",
    [switch]$SkipBuild,
    [switch]$Push
)

$ErrorActionPreference = "Stop"

Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "OpenCog Chocolatey Package Generator" -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host ""

# Determine version
if (-not $Version) {
    if (Test-Path "VERSION") {
        $Version = (Get-Content "VERSION").Trim()
        Write-Host "✓ Version from VERSION file: $Version" -ForegroundColor Green
    } else {
        $Version = (git describe --tags --always 2>$null)
        if (-not $Version) {
            Write-Host "ERROR: Could not determine version!" -ForegroundColor Red
            Write-Host "Please specify version with -Version parameter or create VERSION file" -ForegroundColor Yellow
            exit 1
        }
        Write-Host "✓ Version from git: $Version" -ForegroundColor Green
    }
}

# Determine ZIP path
if (-not $ZipPath) {
    $ZipPath = "dist\opencog-$Version-win64.zip"
}

# Verify ZIP exists or build it
if (-not (Test-Path $ZipPath)) {
    if ($SkipBuild) {
        Write-Host "ERROR: ZIP file not found: $ZipPath" -ForegroundColor Red
        Write-Host "Run build-windows.ps1 first or remove -SkipBuild flag" -ForegroundColor Yellow
        exit 1
    }
    
    Write-Host "ZIP file not found. Building Windows package..." -ForegroundColor Yellow
    & .\build-windows.ps1 -BuildAllComponents
    
    if (-not (Test-Path $ZipPath)) {
        Write-Host "ERROR: Build failed to create ZIP file!" -ForegroundColor Red
        exit 1
    }
}

Write-Host "✓ Using ZIP file: $ZipPath" -ForegroundColor Green

# Calculate SHA256 checksum
Write-Host ""
Write-Host "Calculating SHA256 checksum..." -ForegroundColor Cyan
$Checksum = (Get-FileHash $ZipPath -Algorithm SHA256).Hash
Write-Host "✓ SHA256: $Checksum" -ForegroundColor Green

# Create Chocolatey package directory
$ChocoDir = "chocolatey-package"
if (Test-Path $ChocoDir) {
    Remove-Item -Recurse -Force $ChocoDir
}
New-Item -ItemType Directory -Path $ChocoDir | Out-Null
New-Item -ItemType Directory -Path "$ChocoDir\tools" | Out-Null

Write-Host ""
Write-Host "Creating Chocolatey package files..." -ForegroundColor Cyan

# Copy and update nuspec file
$NuspecContent = Get-Content "packaging\chocolatey\opencog.nuspec" -Raw
$NuspecContent = $NuspecContent -replace '<version>0\.0\.0</version>', "<version>$Version</version>"
$NuspecContent | Out-File -FilePath "$ChocoDir\opencog.nuspec" -Encoding UTF8

# Copy and update install script
$InstallContent = Get-Content "packaging\chocolatey\tools\chocolateyinstall.ps1" -Raw
$InstallContent = $InstallContent -replace '__VERSION__', $Version
$InstallContent = $InstallContent -replace '__CHECKSUM64__', $Checksum
$InstallContent | Out-File -FilePath "$ChocoDir\tools\chocolateyinstall.ps1" -Encoding UTF8

# Copy uninstall script
Copy-Item "packaging\chocolatey\tools\chocolateyuninstall.ps1" "$ChocoDir\tools\"

# Copy VERIFICATION.txt
$VerificationContent = @"
VERIFICATION
Verification is intended to assist the Chocolatey moderators and community
in verifying that this package's contents are trustworthy.

Package can be verified like this:

1. Download the following:
   
   x64: https://github.com/cogpy/occ/releases/download/$Version/opencog-$Version-win64.zip

2. You can use one of the following methods to obtain the SHA256 checksum:
   - Use powershell function 'Get-FileHash'
   - Use Chocolatey utility 'checksum.exe'

   checksum64: $Checksum

File 'LICENSE.txt' is obtained from:
   https://github.com/cogpy/occ/blob/main/LICENSE
"@

$VerificationContent | Out-File -FilePath "$ChocoDir\tools\VERIFICATION.txt" -Encoding UTF8

Write-Host "✓ Package files created" -ForegroundColor Green

# Build Chocolatey package
Write-Host ""
Write-Host "Building Chocolatey package..." -ForegroundColor Cyan

Push-Location $ChocoDir
& choco pack
$PackResult = $LASTEXITCODE
Pop-Location

if ($PackResult -ne 0) {
    Write-Host "ERROR: Chocolatey pack failed!" -ForegroundColor Red
    exit 1
}

$NupkgFile = Get-ChildItem "$ChocoDir\*.nupkg" | Select-Object -First 1

if (-not $NupkgFile) {
    Write-Host "ERROR: No .nupkg file created!" -ForegroundColor Red
    exit 1
}

Write-Host "✓ Chocolatey package created: $($NupkgFile.Name)" -ForegroundColor Green

# Move package to dist directory
if (-not (Test-Path "dist")) {
    New-Item -ItemType Directory -Path "dist" | Out-Null
}

$FinalPath = "dist\$($NupkgFile.Name)"
Move-Item $NupkgFile.FullName $FinalPath -Force

Write-Host ""
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host "Chocolatey Package Complete!" -ForegroundColor Cyan
Write-Host "==================================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Package file: $FinalPath" -ForegroundColor Green
Write-Host "Version: $Version" -ForegroundColor White
Write-Host "SHA256: $Checksum" -ForegroundColor White
Write-Host ""

if ($Push) {
    Write-Host "Pushing to Chocolatey repository..." -ForegroundColor Cyan
    & choco push $FinalPath
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✓ Package pushed successfully!" -ForegroundColor Green
    } else {
        Write-Host "ERROR: Push failed!" -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "To push to Chocolatey, run:" -ForegroundColor Yellow
    Write-Host "  choco push $FinalPath" -ForegroundColor White
    Write-Host ""
    Write-Host "Or run this script with -Push flag" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "To test locally, run:" -ForegroundColor Yellow
Write-Host "  choco install opencog -source `"$((Get-Location).Path)\dist`"" -ForegroundColor White
Write-Host ""
