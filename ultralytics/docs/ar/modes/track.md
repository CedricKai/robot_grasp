---
comments: true
description: تعرف على كيفية استخدام Ultralytics YOLO لتتبع الكائنات في تدفقات الفيديو. أدلة لاستخدام مختلف المتتبعين وتخصيص إعدادات المتتبع.
keywords: Ultralytics، YOLO، تتبع الكائنات، تدفقات الفيديو، BoT-SORT، ByteTrack، دليل Python، دليل خط الأوامر (CLI)
---

# تتبع عدة كائنات باستخدام Ultralytics YOLO

<img width="1024" src="https://user-images.githubusercontent.com/26833433/243418637-1d6250fd-1515-4c10-a844-a32818ae6d46.png" alt="Multi-object tracking examples">

يعد تتبع الكائنات في مجال تحليل الفيديو مهمة حرجة ليس فقط في تحديد موقع وفئة الكائنات داخل الإطار، ولكن أيضًا في الحفاظ على هوية فريدة لكل كائن يتم اكتشافه مع تقدم الفيديو. تكاد التطبيقات لا تعد ولا تحصى - تتراوح من المراقبة والأمان إلى تحليل الرياضة الفورية.

## لماذا يجب اختيار Ultralytics YOLO لتتبع الكائنات؟

إن مخرجات المتتبعين في Ultralytics متسقة مع كشف الكائنات القياسي ولها قيمة مضافة من هويات الكائنات. هذا يجعل من السهل تتبع الكائنات في تدفقات الفيديو وأداء التحليلات التالية. إليك لماذا يجب أن تفكر في استخدام Ultralytics YOLO لتلبية احتياجات تتبع الكائنات الخاصة بك:

- **الكفاءة:** معالجة تدفقات الفيديو في الوقت الحقيقي دون المساومة على الدقة.
- **المرونة:** يدعم العديد من خوارزميات التتبع والتكوينات.
- **سهولة الاستخدام:** واجهة برمجة تطبيقات بسيطة للغاية وخيارات سطر الأوامر للاستدماج السريع والنشر.
- **إمكانية التخصيص:** سهل الاستخدام مع نماذج YOLO مدربة مخصصة، مما يسمح بالاكتمال في التطبيقات ذات النطاق الخاص.

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/hHyHmOtmEgs?si=VNZtXmm45Nb9s-N-"
    title="مشغل فيديو YouTube" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>شاهد:</strong> كشف الكائنات وتتبعها باستخدام Ultralytics YOLOv8.
</p>

## تطبيقات في العالم الحقيقي

|               النقل                |          البيع بالتجزئة          |       الاستزراع المائي       |
|:----------------------------------:|:--------------------------------:|:----------------------------:|
| ![Vehicle Tracking][vehicle track] | ![People Tracking][people track] | ![Fish Tracking][fish track] |
|           تتبع المركبات            |           تتبع الأشخاص           |         تتبع الأسماك         |

## ملامح بلمحة

يوفر Ultralytics YOLO ميزات كشف الكائنات لتوفير تتبع فعال ومتعدد الاستخدامات للكائنات:

- **تتبع فوري:** تتبع الكائنات بسلاسة في مقاطع الفيديو ذات معدل الإطارات العالي.
- **دعم عدة متتبعين:** اختيار بين مجموعة متنوعة من خوارزميات التتبع المعتمدة.
- **تخصيص تكوينات المتتبع المتاحة:** ضبط خوارزمية التتبع لتلبية المتطلبات المحددة عن طريق ضبط مختلف المعلمات.

## متتبعون متاحون

يدعم Ultralytics YOLO الخوارزميات التالية للتتبع. يمكن تمكينها عن طريق تمرير ملف تكوين YAML ذي الصلة مثل "tracker=tracker_type.yaml":

* [BoT-SORT](https://github.com/NirAharon/BoT-SORT) - استخدم `botsort.yaml` لتمكين هذا المتتبع.
* [ByteTrack](https://github.com/ifzhang/ByteTrack) - استخدم `bytetrack.yaml` لتمكين هذا المتتبع.

المتتبع الافتراضي هو BoT-SORT.

## تتبع

لتشغيل المتتبع على تدفقات الفيديو، استخدم نموذج تحديد (Detect) أو قطع (Segment) أو وضع (Pose) مدرب مثل YOLOv8n و YOLOv8n-seg و YOLOv8n-pose.

!!! Example "مثال"

    === "Python"

        ```python
        from ultralytics import YOLO

        # قم بتحميل نموذج رسمي أو مخصص
        model = YOLO('yolov8n.pt')  # قم بتحميل نموذج رسمي Detect
        model = YOLO('yolov8n-seg.pt')  # قم بتحميل نموذج رسمي Segment
        model = YOLO('yolov8n-pose.pt')  # قم بتحميل نموذج رسمي Pose
        model = YOLO('path/to/best.pt')  # قم بتحميل نموذج مخصص مدرب

        # قم بتنفيذ التتبع باستخدام النموذج
        results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True)  # التتبع باستخدام المتتبع الافتراضي
        results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")  # التتبع باستخدام متتبع ByteTrack
        ```

    === "CLI"

        ```bash
        # قم بتنفيذ التتبع باستخدام مختلف النماذج باستخدام واجهة سطر الأوامر
        yolo track model=yolov8n.pt source="https://youtu.be/LNwODJXcvt4"  # نموذج Detect رسمي
        yolo track model=yolov8n-seg.pt source="https://youtu.be/LNwODJXcvt4"  # نموذج Segment رسمي
        yolo track model=yolov8n-pose.pt source="https://youtu.be/LNwODJXcvt4"  # نموذج Pose رسمي
        yolo track model=path/to/best.pt source="https://youtu.be/LNwODJXcvt4"  # تم تدريب نموذج مخصص

        # تتبع عن طريق ByteTrack متتبع
        yolo track model=path/to/best.pt tracker="bytetrack.yaml"
        ```

كما يظهر في الاستخدام أعلاه، يتوفر التتبع لجميع نماذج Detect و Segment و Pose التي تعمل على مقاطع الفيديو أو مصادر البث.

## الاعدادات

### معاملات التتبع

تتشارك إعدادات التتبع الخصائص مع وضع التوقعات (Predict)، مثل `conf` و `iou` و `show`. للحصول على مزيد من التكوينات، راجع صفحة النموذج [Predict](../modes/predict.md#inference-arguments).

!!! Example "مثال"

    === "Python"

        ```python
        from ultralytics import YOLO

        # قم بتكوين معلمات التتبع وقم بتشغيل التتبع
        model = YOLO('yolov8n.pt')
        results = model.track(source="https://youtu.be/LNwODJXcvt4", conf=0.3, iou=0.5, show=True)
        ```

    === "CLI"

        ```bash
        # قم بتكوين معلمات التتبع وقم بتشغيل التتبع باستخدام واجهة سطر الأوامر
        yolo track model=yolov8n.pt source="https://youtu.be/LNwODJXcvt4" conf=0.3, iou=0.5 show
        ```

### اختيار المتتبع

يتيح لك Ultralytics أيضًا استخدام ملف تكوين متتبع معدل. للقيام بذلك، أنقل نسخة من ملف تكوين المتتبع (مثل `custom_tracker.yaml`) من [ultralytics/cfg/trackers](https://github.com/ultralytics/ultralytics/tree/main/ultralytics/cfg/trackers) وقم بتعديل أي تكوينات (باستثناء `tracker_type`) حسب احتياجاتك.

!!! Example "مثال"

    === "Python"

        ```python
        from ultralytics import YOLO

        # قم بتحميل النموذج وتشغيل التتبع باستخدام ملف تكوين مخصص
        model = YOLO('yolov8n.pt')
        results = model.track(source="https://youtu.be/LNwODJXcvt4", tracker='custom_tracker.yaml')
        ```

    === "CLI"

        ```bash
        # قم بتحميل النموذج وتشغيل التتبع باستخدام ملف تكوين مخصص باستخدام واجهة سطر الأوامر
        yolo track model=yolov8n.pt source="https://youtu.be/LNwODJXcvt4" tracker='custom_tracker.yaml'
        ```

للحصول على قائمة شاملة من وسائط تتبع، راجع الصفحة [ultralytics/cfg/trackers](https://github.com/ultralytics/ultralytics/tree/main/ultralytics/cfg/trackers).

## أمثلة Python

### الحفاظ على المسارات التكرارية

فيما يلي سكريبت Python باستخدام OpenCV (cv2) و YOLOv8 لتشغيل تتبع الكائنات على إطارات الفيديو. يفترض هذا السكريبت أنك قد قمت بالفعل بتثبيت الحزم اللازمة (opencv-python و ultralytics). المعامل `persist=True` يخبر المتتبع أن الصورة الحالية أو الإطار التالي في التسلسل ومن المتوقع أن يتوفر مسارات من الصورة السابقة في الصورة الحالية.

!!! Example "For-loop للتدفق مع التتبع"

    ```python
    import cv2
    from ultralytics import YOLO

    # حمّل نموذج YOLOv8
    model = YOLO('yolov8n.pt')

    # افتح ملف الفيديو
    video_path = "path/to/video.mp4"
    cap = cv2.VideoCapture(video_path)

    # تحلق عبر إطارات الفيديو
    while cap.isOpened():
        # قراءة الإطار من الفيديو
        success, frame = cap.read()

        if success:
            # تشغيل تتبع YOLOv8 على الإطار ، وحفظ المسارات بين الإطارات
            results = model.track(frame, persist=True)

            # تصور النتائج على الإطار
            annotated_frame = results[0].plot()

            # عرض الإطار المعلق
            cv2.imshow("YOLOv8 Tracking", annotated_frame)

            # كسر اللوب في حالة الضغط على 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # كسر اللوب في نهاية الفيديو
            break

    # إطلاق كائن التقاط الفيديو وإغلاق نافذة العرض
    cap.release()
    cv2.destroyAllWindows()
    ```

يرجى ملاحظة التغيير من `model(frame)` إلى `model.track(frame)` ، مما يمكن التتبع بدلاً من الكشف البسيط. سيتم تشغيل البرنامج المعدل على كل إطار فيديو وتصور النتائج وعرضها في نافذة. يمكن الخروج من الحلقة عن طريق الضغط على 'q'.

### رسم المسارات عبر الوقت

يمكن أن يوفر رسم المسارات الكائنية عبر الإطارات المتتالية إشارات قيمة حول أنماط الحركة والسلوك للكائنات المكتشفة في الفيديو. باستخدام Ultralytics YOLOv8 ، يعد تصوير هذه المسارات عملية سلسة وفعالة.

في المثال التالي ، نوضح كيفية استخدام قدرات يوكو 8 YOLO لتتبع الكائنات لرسم حركة الكائنات المكتشفة عبر إطارات الفيديو المتعددة. يتضمن هذا البرنامج فتح ملف فيديو وقراءته إطارًا بإطار ، واستخدام نموذج YOLO لتحديد وتتبع العديد من الكائنات. عن طريق الاحتفاظ بنقاط الوسط لمربعات الحدود المكتشفة وتوصيلها ، يمكننا رسم خطوط تمثل المسارات التي تم اتباعها بواسطة الكائنات التي تمت متابعتها.

!!! Example "رسم المسارات عبر إطارات الفيديو المتعددة"

    ```python
    from collections import defaultdict

    import cv2
    import numpy as np

    from ultralytics import YOLO

    # حمّل نموذج YOLOv8
    model = YOLO('yolov8n.pt')

    # افتح ملف الفيديو
    video_path = "path/to/video.mp4"
    cap = cv2.VideoCapture(video_path)

    # احفظ تاريخ المسارات
    track_history = defaultdict(lambda: [])

    # تحلق عبر إطارات الفيديو
    while cap.isOpened():
        # قراءة الإطار من الفيديو
        success, frame = cap.read()

        if success:
            # تشغيل تتبع YOLOv8 على الإطار ، وحفظ المسارات بين الإطارات
            results = model.track(frame, persist=True)

            # الحصول على المربعات ومعرفات المسار
            boxes = results[0].boxes.xywh.cpu()
            track_ids = results[0].boxes.id.int().cpu().tolist()

            # تصور النتائج على الإطار
            annotated_frame = results[0].plot()

            # رسم المسارات
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                track = track_history[track_id]
                track.append((float(x), float(y)))  # x, y نقطة الوسط
                if len(track) > 30:  # احتفظ بـ 90 مسارًا لـ 90 إطارًا
                    track.pop(0)

                # رسم خطوط التتبع
                points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated_frame, [points], isClosed=False, color=(230, 230, 230), thickness=10)

            # عرض الإطار المعلق
            cv2.imshow("YOLOv8 Tracking", annotated_frame)

            # كسر اللوب في حالة الضغط على 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # كسر اللوب في نهاية الفيديو
            break

    # إطلاق كائن التقاط الفيديو وإغلاق نافذة العرض
    cap.release()
    cv2.destroyAllWindows()
    ```

### التتبع متعدد الخيوط

يوفر التتبع متعدد الخيوط القدرة على تشغيل تتبع الكائنات على عدة تدفقات فيديو في وقت واحد. هذا مفيد بشكل خاص عند التعامل مع مدخلات فيديو متعددة ، مثل من كاميرات المراقبة المتعددة ، حيث يمكن أن يعزز المعالجة المتزامنة الكفاءة والأداء بشكل كبير.

في السكريبت البايثون المقدم ، نستخدم وحدة `threading` في Python لتشغيل عدة نسخ متزامنة من المتتبع. يكون لكل موضوع مسؤولية تشغيل المتتبع على ملف فيديو واحد ، وتعمل جميع الخيوط في الخلفية في نفس الوقت.

للتأكد من أن كل خيط يتلقى المعلمات الصحيحة (ملف الفيديو والنموذج المستخدم وفهرس الملف) ، نحدد وظيفة `run_tracker_in_thread` التي تقبل هذه المعلمات وتحتوي على حلقة المتابعة الرئيسية. هذه الوظيفة تقرأ إطار الفيديو الخاصة بالفيديو مباشرة من مصدر الملف الواحد ، وتشغيل المتتبع ، وعرض النتائج.

تستخدم في هذا المثال نموذجين مختلفين: 'yolov8n.pt' و 'yolov8n-seg.pt' ، يقوم كل منهما بتتبع الكائنات في ملف فيديو مختلف. تم تحديد ملفات الفيديو في `video_file1` و `video_file2`.

تعديل معلمات `daemon=True` في `threading.Thread` يعني أن هذه الخيوط ستتم إغلاقها بمجرد انتهاء البرنامج الرئيسي. ثم نبدأ الخيوط باستخدام `start ()` واستخدم `join ()` لجعل الخيط الرئيسي ينتظر حتى ينتهي خيطي المتتبع.

أخيرًا ، بعد اكتمال جميع الخيوط لمهمتها ، يتم إغلاق النوافذ التي تعرض النتائج باستخدام `cv2.destroyAllWindows()`.

!!! Example "Streaming for-loop with tracking"

    ```python
    import threading
    import cv2
    from ultralytics import YOLO


    def run_tracker_in_thread(filename, model, file_index):
        """
        يشغل ملف فيديو أو مصدر تيار الكاميرا بالتزامن مع YOLOv8 النموذج باستخدام تعدد الخيوط.

        هذه الوظيفة تلتقط إطارات الفيديو من ملف أو مصدر الكاميرا المعروف ، وتستخدم نموذج YOLOv8 لتتبع الكائنات.
        يعمل البرنامج في خيطه الخاص للمعالجة المتزامنة.

        Args:
            filename (str): مسار ملف الفيديو أو معرف مصدر كاميرا الويب / خارجية.
            model (obj): كائن نموذج YOLOv8.
            file_index (int): مؤشر لتحديد الملف بشكل فريد ، يُستخدم لأغراض العرض.

        ملاحظة:
            اضغط على 'q' لإنهاء نافذة عرض الفيديو.
        """
        video = cv2.VideoCapture(filename)  # قراءة ملف الفيديو

        while True:
            ret, frame = video.read()  # قراءة إطارات الفيديو

            # إنهاء الدورة إذا لم يتبقى إطارات على الفيديوين
            if not ret:
                break

            # تتبع كائنات في الإطارات إذا توفرت
            results = model.track(frame, persist=True)
            res_plotted = results[0].plot()
            cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

        # إطلاق مصدري الفيديو
        video.release()


    # حمّل النماذج
    model1 = YOLO('yolov8n.pt')
    model2 = YOLO('yolov8n-seg.pt')

    # حدد ملفات الفيديو للمتابعين
    video_file1 = "path/to/video1.mp4"  # مسار ملف الفيديو ، 0 لكاميرا الويب
    video_file2 = 0  # مسار ملف الفيديو ، 0 لكاميرا الويب ، 1 لكاميرا خارجية

    # إنشاء خيوط المتابع
    tracker_thread1 = threading.Thread(target=run_tracker_in_thread, args=(video_file1, model1 ,1), daemon=True)
    tracker_thread2 = threading.Thread(target=run_tracker_in_thread, args=(video_file2, model2, 2), daemon=True)

    # بدء خيوط المتابع
    tracker_thread1.start()
    tracker_thread2.start()

    #  انتظر حتى ينتهي خيط المتابع
    tracker_thread1.join()
    tracker_thread2.join()

    # Clean up and close windows
    cv2.destroyAllWindows()
    ```

يمكن بسهولة توسيع هذا المثال للتعامل مع ملفات فيديو ونماذج أخرى من خلال إنشاء مزيد من الخيوط وتطبيق نفس المنهجية.

## المساهمة في المتتبعون الجديدون

هل أنت ماهر في التتبع متعدد الكائنات وقد نفذت أو صيغت بنجاح خوارزمية تتبع باستخدام Ultralytics YOLO؟ ندعوك للمشاركة في قسم المتتبعين لدينا في [ultralytics/cfg/trackers](https://github.com/ultralytics/ultralytics/tree/main/ultralytics/cfg/trackers)! قد تكون التطبيقات في العالم الحقيقي والحلول التي تقدمها لا تقدر بثمن للمستخدمين العاملين على مهام التتبع.

من خلال المساهمة في هذا القسم ، تساعد في توسيع نطاق حلول التتبع المتاحة في إطار Ultralytics YOLO ، مضيفًا طبقة أخرى من الوظائف والفعالية للمجتمع.

لبدء المساهمة ، يرجى الرجوع إلى [دليل المساهمة الخاص بنا](https://docs.ultralytics.com/help/contributing) للحصول على تعليمات شاملة حول تقديم طلب سحب (PR) 🛠️. نتطلع بشكل كبير إلى ما ستجلبه للطاولة!

لنعزز معًا قدرات عملية التتبع لأجهزة Ultralytics YOLO 🙏!

[vehicle track]: https://github.com/RizwanMunawar/ultralytics/assets/62513924/ee6e6038-383b-4f21-ac29-b2a1c7d386ab

[people track]:  https://github.com/RizwanMunawar/ultralytics/assets/62513924/93bb4ee2-77a0-4e4e-8eb6-eb8f527f0527

[fish track]:    https://github.com/RizwanMunawar/ultralytics/assets/62513924/a5146d0f-bfa8-4e0a-b7df-3c1446cd8142
