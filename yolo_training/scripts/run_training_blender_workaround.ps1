param(
    [string]$PythonExe = "C:\Program Files\Blender Foundation\Blender 5.0\5.0\python\bin\python.exe",
    [string]$OldSitePackages = "C:\Users\USER\Desktop\OKUL\Pamuk Projesi\YoloTest1\venv\Lib\site-packages",
    [string]$Data = "configs\data.yaml",
    [string]$Model = "weights\yolo11n.pt",
    [int]$Epochs = 80,
    [int]$ImgSize = 512,
    [int]$Batch = 4,
    [string]$Device = "0",
    [string]$Name = "cotton_v5_yolo11n_512_b4_e80"
)

$Root = Split-Path -Parent $PSScriptRoot
$env:PYTHONPATH = $OldSitePackages
$env:YOLO_CONFIG_DIR = Join-Path $Root ".ultralytics"
New-Item -ItemType Directory -Force -Path $env:YOLO_CONFIG_DIR | Out-Null

& $PythonExe (Join-Path $Root "src\train.py") `
    --data $Data `
    --model $Model `
    --epochs $Epochs `
    --imgsz $ImgSize `
    --batch $Batch `
    --workers 0 `
    --device $Device `
    --name $Name

exit $LASTEXITCODE
