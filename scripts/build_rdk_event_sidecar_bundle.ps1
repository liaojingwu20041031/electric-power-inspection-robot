param(
    [string]$OutputDirectory = "",
    [switch]$Force
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$commit = (git -C $repo rev-parse --short=8 HEAD).Trim()
$gitStatus = @(git -C $repo status --porcelain=v1 --untracked-files=all)
if ($gitStatus.Count -gt 0) {
    throw "Refusing to package an uncommitted source tree. Commit or stash changes first."
}
if (-not $OutputDirectory) {
    $OutputDirectory = Join-Path (Split-Path $repo -Parent) "artifacts\rdk_event_sidecar"
}
$OutputDirectory = [System.IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Force -Path $OutputDirectory | Out-Null
$stage = Join-Path $OutputDirectory "rdk_event_sidecar_$commit"
$archive = "$stage.tar.gz"
$checksum = "$archive.sha256"
if ((Test-Path $stage) -or (Test-Path $archive)) {
    if (-not $Force) {
        throw "Output already exists. Use -Force or select another -OutputDirectory: $stage"
    }
    Remove-Item -LiteralPath $stage -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $archive -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $checksum -Force -ErrorAction SilentlyContinue
}
New-Item -ItemType Directory -Path $stage | Out-Null

$files = @(
    @{ Source = "src\ylhb_base\scripts\event_feature_node.py"; Target = "event_feature_node.py" },
    @{ Source = "src\ylhb_base\scripts\event_feature_core.py"; Target = "event_feature_core.py" },
    @{ Source = "src\ylhb_base\scripts\pointcloud_reader.py"; Target = "pointcloud_reader.py" },
    @{ Source = "src\ylhb_base\scripts\auxiliary_sensor_data.py"; Target = "auxiliary_sensor_data.py" },
    @{ Source = "src\ylhb_base\scripts\auxiliary_sensor_replay_node.py"; Target = "auxiliary_sensor_replay_node.py" },
    @{ Source = "src\ylhb_base\scripts\ros_shutdown.py"; Target = "ros_shutdown.py" },
    @{ Source = "scripts\inspect_event_topic.py"; Target = "inspect_event_topic.py" },
    @{ Source = "scripts\run_rdk_event_feature_sidecar.sh"; Target = "run_rdk_event_feature_sidecar.sh" },
    @{ Source = "scripts\run_rdk_event_csv_replay.sh"; Target = "run_rdk_event_csv_replay.sh" },
    @{ Source = "scripts\probe_rdk_x5_event_camera.sh"; Target = "probe_rdk_x5_event_camera.sh" },
    @{ Source = "scripts\rdk_env.sh"; Target = "rdk_env.sh" },
    @{ Source = "docs\dg202611\RDK_X5_EVENT_CAMERA_START.md"; Target = "README.md" }
)
foreach ($item in $files) {
    Copy-Item -LiteralPath (Join-Path $repo $item.Source) -Destination (Join-Path $stage $item.Target)
}

$recordPath = Join-Path $stage "BUNDLE_RECORD.json"
$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
$recordJson = @{
    schema_version = 1
    source_commit = (git -C $repo rev-parse HEAD).Trim()
    source_branch = (git -C $repo branch --show-current).Trim()
    created_local = (Get-Date).ToString("o")
    target = "D-Robotics RDK X5 / TROS Humble"
    publishes_motion_commands = $false
    payload_files = @($files.Target)
    checksum_manifest = "manifest.sha256"
} | ConvertTo-Json -Depth 4
$recordJson = $recordJson -replace "`r`n", "`n"
[System.IO.File]::WriteAllText($recordPath, $recordJson + "`n", $utf8NoBom)

$manifestRows = foreach ($file in Get-ChildItem -LiteralPath $stage -File | Sort-Object Name) {
    $hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $file.FullName).Hash.ToLowerInvariant()
    "$hash  $($file.Name)"
}
$manifestText = ($manifestRows -join "`n") + "`n"
[System.IO.File]::WriteAllText((Join-Path $stage "manifest.sha256"), $manifestText, [System.Text.Encoding]::ASCII)

Push-Location $OutputDirectory
try {
    tar -czf (Split-Path -Leaf $archive) (Split-Path -Leaf $stage)
} finally {
    Pop-Location
}
$archiveHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
$checksumText = "$archiveHash  $(Split-Path -Leaf $archive)`n"
[System.IO.File]::WriteAllText($checksum, $checksumText, [System.Text.Encoding]::ASCII)
Write-Output "RDK_EVENT_SIDECAR_DIR=$stage"
Write-Output "RDK_EVENT_SIDECAR_ARCHIVE=$archive"
Write-Output "RDK_EVENT_SIDECAR_SHA256=$archiveHash"
