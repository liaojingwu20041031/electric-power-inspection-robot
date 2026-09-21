[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9.-]+$')]
    [string]$RobotHost,

    [ValidatePattern('^[A-Za-z0-9._-]+$')]
    [string]$RobotUser = 'nvidia',

    [string]$ArchivePath,

    [string]$Commit,

    [string]$ExpectedSha256
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = Split-Path -Parent $PSScriptRoot
$workspaceRoot = Split-Path -Parent $repositoryRoot
$archivePathProvided = -not [string]::IsNullOrWhiteSpace($ArchivePath)
if (-not $Commit) {
    if ($archivePathProvided) {
        $archiveLeaf = [IO.Path]::GetFileName($ArchivePath)
        $archiveCommit = $null
        if ($archiveLeaf -match '^(?:dg202611_ws|dg202611_robot_integration)_([0-9a-f]{7,40})\.tar\.gz$') {
            $archiveCommit = $Matches[1]
        }
        if (-not $archiveCommit) {
            throw "Cannot infer the commit from archive name '$archiveLeaf'. Pass -Commit explicitly."
        }
        $Commit = $archiveCommit
    }
    else {
        $Commit = (& git -C $repositoryRoot rev-parse --short HEAD).Trim()
        if ($LASTEXITCODE -ne 0) {
            throw 'Unable to read the integration commit id.'
        }
    }
}
if ($Commit -notmatch '^[0-9a-f]{7,40}$') {
    throw "Invalid commit id: $Commit"
}
if (-not $ArchivePath) {
    # The current release builder writes dg202611_ws_<commit>.tar.gz at the
    # workspace root. Keep the older artifacts naming as a compatibility
    # fallback so a documented default invocation still finds the package.
    $archiveCandidates = @(
        (Join-Path $workspaceRoot "dg202611_ws_$Commit.tar.gz"),
        (Join-Path $workspaceRoot "artifacts\dg202611_robot_integration_$Commit.tar.gz"),
        (Join-Path $workspaceRoot "artifacts\dg202611_ws_$Commit.tar.gz")
    )
    $ArchivePath = $archiveCandidates | Where-Object {
        Test-Path -LiteralPath $_ -PathType Leaf
    } | Select-Object -First 1
    if (-not $ArchivePath) {
        throw "No release archive found for commit $Commit. Expected one of: $($archiveCandidates -join ', ')"
    }
}
$archiveLeaf = [IO.Path]::GetFileName($ArchivePath)
if ($archivePathProvided -and $archiveLeaf -match '^(?:dg202611_ws|dg202611_robot_integration)_([0-9a-f]{7,40})\.tar\.gz$') {
    $archiveCommit = $Matches[1]
    $commitMatchesArchive = ($archiveCommit -eq $Commit) -or $archiveCommit.StartsWith($Commit) -or $Commit.StartsWith($archiveCommit)
    if (-not $commitMatchesArchive) {
        throw "Commit '$Commit' does not match archive filename commit '$archiveCommit'."
    }
}
$archive = (Resolve-Path -LiteralPath $ArchivePath).Path
$installer = Join-Path $PSScriptRoot 'install_release_on_robot.sh'
if (-not (Test-Path -LiteralPath $installer -PathType Leaf)) {
    throw "Robot installer is missing: $installer"
}

if (-not $ExpectedSha256) {
    $checksumFile = "$archive.sha256"
    if (-not (Test-Path -LiteralPath $checksumFile -PathType Leaf)) {
        throw "Checksum sidecar is missing: $checksumFile"
    }
    $ExpectedSha256 = ((Get-Content -LiteralPath $checksumFile -Raw).Trim() -split '\s+')[0]
}
if ($ExpectedSha256 -notmatch '^[0-9A-Fa-f]{64}$') {
    throw "Invalid expected SHA256: $ExpectedSha256"
}
$actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToUpperInvariant()
if ($actualHash -ne $ExpectedSha256.ToUpperInvariant()) {
    throw "Archive SHA256 mismatch. Expected $ExpectedSha256, got $actualHash"
}

if (-not (Test-NetConnection -ComputerName $RobotHost -Port 22 -InformationLevel Quiet)) {
    throw "SSH is not reachable at ${RobotHost}:22. Connect the computer and robot to the same network first."
}

$destination = "$RobotUser@$RobotHost"
$remoteArchiveName = [IO.Path]::GetFileName($archive)
$releaseName = "dg202611_ws_$Commit"

Write-Host "Verified archive: $archive"
Write-Host "Uploading to $destination without changing /home/nvidia/ros2_DL ..."
& scp -- $archive $installer "${destination}:/home/nvidia/"
if ($LASTEXITCODE -ne 0) {
    throw "scp failed with exit code $LASTEXITCODE"
}

$remoteCommand = "bash /home/nvidia/install_release_on_robot.sh /home/nvidia/$remoteArchiveName $releaseName $Commit $($ExpectedSha256.ToLowerInvariant())"
& ssh -- $destination $remoteCommand
if ($LASTEXITCODE -ne 0) {
    throw "Robot preflight failed with exit code $LASTEXITCODE. The original workspace was not modified."
}

Write-Host "Deployment and preflight completed: /home/nvidia/$releaseName"
Write-Host 'No motion command was sent.'
