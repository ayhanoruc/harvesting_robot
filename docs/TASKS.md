cool, now lets get back to our tasks, we want to achieve:\

- add the 3D models of cotton clusters to the world. clone https://github.com/denzo-ferrari/Cotton-Tracking-YOLO/  with a side note for 3D model: İki farklı fotodan oluşturduğumuz GLB modellerini Object0 ve Object1 olarak repoya ekledik, hangisi uygunsa onu kullanın
    - GLB is now supported in new fortess version
    - Repository Structure
        
        Cotton-Tracking-YOLO/
        ├── [best.pt](http://best.pt/)                     # Trained YOLO model (cotton detection)
        ├── object_0.glb               # 3D cotton cluster model #1
        ├── object_1.glb               # 3D cotton cluster model #2
        ├── sticky_tracker.yaml        # BEST: BoT-SORT "sticky" tracking config
        ├── custom_tracker.yaml        # ByteTrack alternative
        ├── robust_botsort.yaml        # BoT-SORT "shape shifter" variant
        ├── track_webcam_v3.py        # BEST: Live webcam tracking
        ├── track_mp4_v2.py           # BEST: Video file tracking
        ├── [train.py](http://train.py/)                   # Training script (Roboflow + YOLO11)
        └── Cotton-boll-and-cluster-2/ # Training dataset from Roboflow
        └── data.yaml              # 2 classes: cotton_boll, cotton_boll-cluster
        
        - nc: 2 , names: ['cotton_boll', 'cotton_boll-cluster']
    
    ---
    
    Tracker Comparison
    
    | Config | Type | track_buffer | Key Feature |
    | --- | --- | --- | --- |
    | sticky_tracker.yaml | BoT-SORT | 120 (4s) | Best for stable IDs - with Re-ID |
    | robust_botsort.yaml | BoT-SORT | 120 | Handles shape changes (lower match_thresh) |
    | custom_tracker.yaml | ByteTrack | 60 | Simpler, no Re-ID |
    
    Recommended: sticky_tracker.yaml with:
    
    - with_reid: True - Visual appearance matching
    - track_buffer: 120 - 4-second memory for ID persistence
    - fuse_score: True - Critical for crash prevention



- lets have the YOLO running succesfully in our environment: Repoya README eklemedik ama en iyi çalışan dosyalar track_webcam_v3 ve track_mp4_v2 (ortak config: sticky_track.yaml); geri kalan dosyalar eski denemeler veya anlık testler için, biraz dağınık oldu.
    1. Multi-Frame Cluster Identification
        - YOLO detects per-frame, doesn't track identity across frames
        - You'll need a tracker layer on top (e.g., DeepSORT, ByteTrack)
- camera RGB + depth -> YOLO BOX bounding(partial-or-complete) -> gives pixels, we choose the center of the detected box -> correct the view one step -> YOLO BOX bounding(partial-or-complete) -> gives pixels, we
choose the center of the detected box -> form X,Y,Z coordinates : this task now requires a definite validation like:\
we already know the object's(cotton) position in 3D(global frame'e göre, absolute olmalı çünkü kameraya göre yaparsak karışır. bu hesapta kameranın 3D state'i + relative bilgileri vs katıp yapılır sanırım.) -> we
can run our this custom YoloSpatialDetectionPipeline, and compare to the ground truth with a margin of error.\
    1. Coordinate Transform Chain
    pixel (u,v) + depth → camera_frame (x,y,z) → base_link → world
        - Requires: camera intrinsics (K matrix), TF chain must be accurate
        - Depth at center pixel may not be reliable (edges, noise) - consider small ROI averaging
- explore yerine, şöyle yapsak?: home positiondan başlar, geniş açıda: kafayı sola sağa, robust perceptionda explore patthte yaptıgımız gibi çevirir -> burda önümüzdeki tüm alanı taramalıyız final yargıya varmadan
önce. bu scan’i geometrik olarak nasıl saglarız?\\
    1. Scanning Geometry for 4-DOF
        - Your arm has limited DOF for arbitrary camera poses
        - With wrist-mounted camera, scanning by joint1 (base rotation) + joint2/3 (tilt) gives coverage
        - Need to calculate FOV overlap to ensure no blind spots
- bu exploration bizim home view'imiz, burda cluster detection+identification(labeling) yapmalı YOLO modelimiz, yani cluster_1,2,3ü tanımalı kamera açıları değiştikçe. bunlar nasıl kayıtlı tutuluyor ve bilgilerine erişiliyor hafızada ögrenmeliyiz, çünkü bunlar kod içindeki harvesting loopta process edilecek teker teker. burda tutmak istediğimiz bilgiler home view’den bakıştaki 3D coordinateler(neden 2 iterasyon yapmıyoruz→ çünkü muhtemelen zaten cluster full görünecek, error correction yapmaya gerek yok)
    - "Cluster Fully Visible" Criterion
        - How do you detect partial vs full visibility?
        - bounding box touching image edge, confidence threshold,
- son olarak da research repolarında gördüğümüz robust 3D point approach ve pick-and-place sistemlerini entegre etmeye çalışacagız.
- bunları component olarak implemente edeblirsek braccio 6DOF urdf’ini entegre edeceğiz.