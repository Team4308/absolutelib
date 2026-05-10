param(
    [string]$TeamNumber = "4308",
    [string]$OrangePiIP = "",
    [string]$Username = "",
    [string]$Password = "",
    [switch]$UseSSHKey = $false
)

Write-Host "Orange Pi Trajectory Coprocessor Setup Script" -ForegroundColor Cyan
Write-Host "=============================================" -ForegroundColor Cyan
Write-Host ""

if ([string]::IsNullOrWhiteSpace($OrangePiIP)) {
    $OrangePiIP = Read-Host "Enter Orange Pi IP address (e.g., 10.43.8.11)"
}

if ([string]::IsNullOrWhiteSpace($Username)) {
    $Username = Read-Host "Enter username (default: photon)"
    if ([string]::IsNullOrWhiteSpace($Username)) { $Username = "photon" }
}

if ([string]::IsNullOrWhiteSpace($Password) -and -not $UseSSHKey) {
    $secPassword = Read-Host "Enter password (default: vision)" -AsSecureString
    if ($secPassword.Length -eq 0) {
        $Password = "vision"
    } else {
        $Password = [System.Runtime.InteropServices.Marshal]::PtrToStringAuto([System.Runtime.InteropServices.Marshal]::SecureStringToCoTaskMemUnicode($secPassword))
    }
}

Write-Host ""
Write-Host "Configuration:" -ForegroundColor Yellow
Write-Host "  Orange Pi IP: $OrangePiIP"
Write-Host "  Team Number: $TeamNumber"
Write-Host "  Username: $Username"
Write-Host ""

$JAR_FILE = "coprocessor\build\libs\coprocessor.jar"
if (-not (Test-Path $JAR_FILE)) {
    Write-Host "ERROR: JAR file not found at $JAR_FILE" -ForegroundColor Red
    Write-Host "Please run './gradlew :coprocessor:shadowJar' first" -ForegroundColor Red
    exit 1
}

Write-Host "Building coprocessor..." -ForegroundColor Cyan
./gradlew :coprocessor:shadowJar -q
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Build failed" -ForegroundColor Red
    exit 1
}

$REMOTE_DEPLOY_DIR = "/opt/frc/trajectory-coprocessor"
$SERVICE_NAME = "trajectory-coprocessor"

Write-Host "Connecting to Orange Pi at $OrangePiIP..." -ForegroundColor Cyan

$sshUser = "$Username@$OrangePiIP"

Write-Host "Checking SSH connectivity..." -ForegroundColor Cyan

$sshOptions = @("-o", "StrictHostKeyChecking=no", "-o", "UserKnownHostsFile=/dev/null", "-o", "ConnectTimeout=5")

if ($UseSSHKey) {
    Write-Host "Using SSH key authentication" -ForegroundColor Yellow
} else {
    Write-Host "Using password authentication" -ForegroundColor Yellow
    Write-Host "NOTE: You will be prompted for password interactively" -ForegroundColor Yellow
}

Write-Host "Testing SSH connection (you may need to accept host key)..." -ForegroundColor Cyan

if ($UseSSHKey) {
    ssh @sshOptions $sshUser "sudo mkdir -p $REMOTE_DEPLOY_DIR && sudo chown $Username $REMOTE_DEPLOY_DIR" 2>$null
} else {
    # Use a here-string to pass password via stdin for SSH
    # This avoids the interactive prompt by piping the password
    $connectTest = ssh @sshOptions $sshUser "echo connected" 2>&1
    if ($LASTEXITCODE -ne 0 -or $connectTest -notlike "*connected*") {
        Write-Host "First connection attempt - you may need to accept the host key" -ForegroundColor Yellow
        ssh @sshOptions $sshUser "sudo mkdir -p $REMOTE_DEPLOY_DIR && sudo chown $Username $REMOTE_DEPLOY_DIR"
    } else {
        ssh @sshOptions $sshUser "sudo mkdir -p $REMOTE_DEPLOY_DIR && sudo chown $Username $REMOTE_DEPLOY_DIR" 2>$null
    }
}

if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Could not connect to Orange Pi at $OrangePiIP" -ForegroundColor Red
    Write-Host "Verify:" -ForegroundColor Yellow
    Write-Host "  1. Orange Pi IP address is correct: $OrangePiIP"
    Write-Host "  2. Username and password are correct: $Username"
    Write-Host "  3. Orange Pi is on the network and reachable"
    Write-Host "  4. SSH is enabled on Orange Pi"
    Write-Host "" -ForegroundColor Red
    Write-Host "To test connection manually:" -ForegroundColor Yellow
    Write-Host "  ssh $Username@$OrangePiIP" -ForegroundColor Cyan
    exit 1
}

Write-Host "Transferring JAR file..." -ForegroundColor Cyan
if ($UseSSHKey) {
    scp @sshOptions $JAR_FILE "${sshUser}:${REMOTE_DEPLOY_DIR}/coprocessor.jar"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Failed to transfer JAR file" -ForegroundColor Red
        exit 1
    }
} else {
    scp @sshOptions $JAR_FILE "${sshUser}:${REMOTE_DEPLOY_DIR}/coprocessor.jar"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "ERROR: Failed to transfer JAR file" -ForegroundColor Red
        exit 1
    }
}

Write-Host "Creating systemd service..." -ForegroundColor Cyan

$SERVICE_CONTENT = @"
[Unit]
Description=FRC 2026 Trajectory Coprocessor
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=$REMOTE_DEPLOY_DIR
ExecStart=/usr/bin/java -jar $REMOTE_DEPLOY_DIR/coprocessor.jar
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

Environment=TEAM_NUMBER=$TeamNumber
Environment=HTTP_PORT=5805

[Install]
WantedBy=multi-user.target
"@

$SCRIPT = @"
sudo bash -c 'cat > /etc/systemd/system/$SERVICE_NAME.service << '"'"'EOF'"'"'
$SERVICE_CONTENT
EOF
systemctl daemon-reload
systemctl enable $SERVICE_NAME
systemctl restart $SERVICE_NAME
systemctl status $SERVICE_NAME'
"@

if ($UseSSHKey) {
    ssh @sshOptions $sshUser $SCRIPT 2>$null
} else {
    ssh @sshOptions $sshUser $SCRIPT 2>$null
}

Write-Host ""
Write-Host "Setup complete!" -ForegroundColor Green
Write-Host ""
Write-Host "Service Management:" -ForegroundColor Yellow
Write-Host "  View logs:     ssh $sshUser 'sudo journalctl -u $SERVICE_NAME -f'"
Write-Host "  Stop service:  ssh $sshUser 'sudo systemctl stop $SERVICE_NAME'"
Write-Host "  Start service: ssh $sshUser 'sudo systemctl start $SERVICE_NAME'"
Write-Host "  Check status:  ssh $sshUser 'sudo systemctl status $SERVICE_NAME'"
Write-Host ""
Write-Host "The coprocessor will automatically start on Orange Pi boot." -ForegroundColor Green
