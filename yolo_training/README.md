# Cotton YOLO Train and Inference Package

Clean package date: 2026-06-05

This folder keeps the latest cotton detection model, the reproducible train/test commands, and the older live tracking experiments in one readable place.

## Folder map

```text
configs/
  data.yaml                  Latest local dataset config
  data_template.yaml         Portable template if the dataset is copied into dataset/
weights/
  best.pt                    Latest trained model for Ayhan / ROS2
  yolo11n.pt                 Base model used for training, if available locally
metrics/
  results.csv                Training metrics per epoch
  results.png                Train/val curves
  confusion_matrix*.png      Validation/test confusion matrices
  P_curve.png, R_curve.png, F1_curve.png, PR_curve.png
sample_predictions/
  Example outputs from the latest model
src/
  train.py                   Main training entrypoint
  validate.py                Val/test metrics entrypoint
  predict.py                 Image/video/folder/webcam inference
  summarize_results.py       Quick summary of results.csv
  exp/                       Older webcam/video/live tracking experiments
scripts/
  run_training_blender_workaround.ps1
```

## Latest model summary

- Model: YOLO11n detection
- Classes: `0=cotton_boll`, `1=unripe-cotton`
- Train split: 603 images
- Validation split: 57 images
- Test split: 29 images
- Training: 80 epochs, image size 512, batch 4
- GPU used: RTX 3050 Ti Laptop GPU
- Recommended inference confidence: start at `0.54`

Latest test result:

| class | precision | recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| all | 0.956 | 0.971 | 0.972 | 0.806 |
| cotton_boll | 0.913 | 0.946 | 0.950 | 0.811 |
| unripe-cotton | 1.000 | 0.995 | 0.995 | 0.801 |

The model is strong enough for the ROS2 demo pipeline. For harvesting logic, treat `cotton_boll` as the pickable class and usually ignore `unripe-cotton`.

## 1. Environment setup

Open PowerShell in this folder:

```powershell
cd "C:\Users\USER\Desktop\OKUL\Pamuk Projesi\YOLO_train_inference_2026-06-05_release"
python -m venv .venv
.\.venv\Scripts\activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

For GPU training, make sure PyTorch sees CUDA:

```powershell
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

If it prints `False`, install the CUDA build of PyTorch for your driver/CUDA version, then run the install command again.

## 2. Dataset setup

The included `configs/data.yaml` points to the latest local dataset folder:

```yaml
path: ../../YOLO_2026-05-30
```

If someone receives only this clean folder, unzip the Roboflow YOLO dataset into a folder named `dataset/`, then either:

1. Use `configs/data_template.yaml`, or
2. Edit `configs/data.yaml` and change only the `path:` line.

Expected dataset layout:

```text
dataset/
  train/images
  train/labels
  valid/images
  valid/labels
  test/images
  test/labels
```

## 3. Train from scratch/fine tune

Default local training command:

```powershell
python src\train.py --data configs\data.yaml --model weights\yolo11n.pt --epochs 80 --imgsz 512 --batch 4 --device 0 --name cotton_v5_yolo11n_512_b4_e80
```

Outputs go here:

```text
runs/detect/cotton_v5_yolo11n_512_b4_e80/
  weights/best.pt
  weights/last.pt
  results.csv
```

Use `best.pt` for ROS2/inference, not `last.pt`, unless you have a specific reason.

## 4. Test/validate the model

Run on the test split:

```powershell
python src\validate.py --model weights\best.pt --data configs\data.yaml --split test --imgsz 512 --device 0 --plots
```

Run on validation split:

```powershell
python src\validate.py --model weights\best.pt --data configs\data.yaml --split val --imgsz 512 --device 0 --plots
```

Ultralytics will create confusion matrices and curves under `runs/val/...`.

To summarize the stored training curves:

```powershell
python src\summarize_results.py --results metrics\results.csv
```

## 5. Inference

Single image or folder:

```powershell
python src\predict.py --model weights\best.pt --source "path\to\image_or_folder" --conf 0.54 --save-txt --save-conf
```

Video:

```powershell
python src\predict.py --model weights\best.pt --source "path\to\video.mp4" --conf 0.54
```

Webcam:

```powershell
python src\predict.py --model weights\best.pt --source 0 --conf 0.54 --show
```

Outputs go under `runs/predict/...`.

Confidence guidance:

- `0.50-0.55`: balanced start point, best for field testing
- `0.60-0.70`: fewer false positives, but may miss more cotton
- `0.85+`: too strict for this project unless the camera view is very stable

## 6. Files for Ayhan / ROS2

Send or copy this file:

```text
weights/best.pt
```

Class map:

```text
0 -> cotton_boll
1 -> unripe-cotton
```

ROS2 node should load `best.pt`, publish detections from YOLO, and only send pick targets for class `cotton_boll`. If any older ROS2 code expects `cotton_boll-cluster`, update it to `cotton_boll`.

## 7. Old live tracking experiments

Old webcam/video tracking scripts are stored under:

```text
src/exp/
```

They are kept for reference. Use `src/predict.py` first for clean inference. Use the exp scripts only when you want to revisit live tracking/trails/BOTSort style behavior.

## 8. Local workaround

If normal Python has dependency issues on this laptop, use the preserved Blender Python workaround:

```powershell
powershell -ExecutionPolicy Bypass -File scripts\run_training_blender_workaround.ps1
```

This is machine-specific and should not be the first choice on a new computer.
