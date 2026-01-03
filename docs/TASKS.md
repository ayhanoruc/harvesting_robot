cool, now lets get back to our tasks, we want to achieve:\

- add the 3D models of cotton clusters to the world. clone https://github.com/denzo-ferrari/Cotton-Tracking-YOLO/  with a side note for 3D model: İki farklı fotodan oluşturduğumuz GLB modellerini Object0 ve Object1 olarak repoya ekledik, hangisi uygunsa onu kullanın
    1. GLB to Gazebo Conversion
        - Gazebo uses SDF/URDF with DAE/STL meshes, not GLB directly
        - Need: GLB → GLTF → DAE conversion (Blender or gltf2dae tool)
        - Or use gz-sim 8+ which has native GLTF support (check your Gazebo version)
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