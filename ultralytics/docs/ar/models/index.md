---
comments: true
description: استكشف مجموعة متنوعة من عائلة YOLO، ونماذج SAM وMobileSAM وFastSAM وYOLO-NAS وRT-DETR المدعومة من Ultralytics. ابدأ بأمثلة لكل من استخدام واجهة الأوامر وPython.
keywords: Ultralytics, documentation, YOLO, SAM, MobileSAM, FastSAM, YOLO-NAS, RT-DETR, models, architectures, Python, CLI
---

# النماذج المدعومة من Ultralytics

أهلاً بك في وثائق نماذج Ultralytics! نحن نقدم الدعم لمجموعة واسعة من النماذج، كل منها مُصمم لمهام محددة مثل [الكشف عن الأجسام](../tasks/detect.md)، [تقطيع الحالات](../tasks/segment.md)، [تصنيف الصور](../tasks/classify.md)، [تقدير الوضعيات](../tasks/pose.md)، و[تتبع الأجسام المتعددة](../modes/track.md). إذا كنت مهتمًا بالمساهمة في هندسة نموذجك مع Ultralytics، راجع دليل [المساهمة](../../help/contributing.md).

!!! Note "ملاحظة"

    🚧 تحت الإنشاء: وثائقنا بلغات متعددة قيد الإنشاء حاليًا، ونحن نعمل بجد لتحسينها. شكرًا لصبرك! 🙏

## النماذج المميزة

إليك بعض النماذج الرئيسية المدعومة:

1. **[YOLOv3](yolov3.md)**: الإصدار الثالث من عائلة نموذج YOLO، الذي أنشأه أصلاً Joseph Redmon، والمعروف بقدراته الفعالة في الكشف عن الأجسام في الوقت الفعلي.
2. **[YOLOv4](yolov4.md)**: تحديث محلي لـ YOLOv3، تم إصداره بواسطة Alexey Bochkovskiy في 2020.
3. **[YOLOv5](yolov5.md)**: نسخة مُحسنة من هندسة YOLO من قبل Ultralytics، توفر أداءً أفضل وتوازن في السرعة مقارنة بالإصدارات السابقة.
4. **[YOLOv6](yolov6.md)**: أُصدرت بواسطة [Meituan](https://about.meituan.com/) في 2022، ويُستخدم في العديد من روبوتات التوصيل الذاتية للشركة.
5. **[YOLOv7](yolov7.md)**: تم إصدار نماذج YOLO المحدثة في 2022 بواسطة مؤلفي YOLOv4.
6. **[YOLOv8](yolov8.md) جديد 🚀**: الإصدار الأحدث من عائلة YOLO، يتميز بقدرات مُعززة مثل تقطيع الحالات، تقدير الوضعيات/النقاط الرئيسية، والتصنيف.
7. **[Segment Anything Model (SAM)](sam.md)**: نموذج Segment Anything Model (SAM) من Meta.
8. **[Mobile Segment Anything Model (MobileSAM)](mobile-sam.md)**: نموذج MobileSAM للتطبيقات المحمولة، من جامعة Kyung Hee.
9. **[Fast Segment Anything Model (FastSAM)](fast-sam.md)**: نموذج FastSAM من مجموعة تحليل الصور والفيديو، والمعهد الصيني للأتمتة، وأكاديمية العلوم الصينية.
10. **[YOLO-NAS](yolo-nas.md)**: نماذج YOLO Neural Architecture Search (NAS).
11. **[Realtime Detection Transformers (RT-DETR)](rtdetr.md)**: نماذج Realtime Detection Transformer (RT-DETR) من PaddlePaddle التابعة لشركة Baidu.

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/MWq1UxqTClU?si=nHAW-lYDzrz68jR0"
    title="مشغل فيديو YouTube" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>شاهد:</strong> تشغيل نماذج YOLO من Ultralytics في بضعة أسطر من الكود فقط.
</p>

## البدء في الاستخدام: أمثلة على الاستخدام

يوفر هذا المثال أمثلة مبسطة على التدريب والاستدلال باستخدام YOLO. للحصول على الوثائق الكاملة عن هذه وغيرها من [الأوضاع](../modes/index.md), انظر صفحات وثائق [التنبؤ](../modes/predict.md)، و[التدريب](../modes/train.md)، و[التقييم](../modes/val.md) و[التصدير](../modes/export.md).

لاحظ أن المثال أدناه هو لنماذج [Detect](../tasks/detect.md) YOLOv8 لكشف الأجسام. للاطلاع على المهام الإضافية المدعومة، راجع وثائق [Segment](../tasks/segment.md)، و[Classify](../tasks/classify.md) و[Pose](../tasks/pose.md).

!!! Example "مثال"

    === "Python"

        نماذج `*.pt` المُدربة مسبقًا وملفات الإعداد `*.yaml` يمكن أن تُمرر إلى فئات `YOLO()`, `SAM()`, `NAS()` و `RTDETR()` لإنشاء مثال نموذج في Python:

        ```python
        من ultralytics استيراد YOLO

        # تحميل نموذج YOLOv8n المُدرب مسبقًا على COCO
        النموذج = YOLO('yolov8n.pt')

        # عرض معلومات النموذج (اختياري)
        model.info()

        # تدريب النموذج على مجموعة البيانات المثالية COCO8 لمدة 100 عصر
        النتائج = model.train(data='coco8.yaml', epochs=100, imgsz=640)

        # تشغيل الاستدلال بنموذج YOLOv8n على صورة 'bus.jpg'
        النتائج = model('path/to/bus.jpg')
        ```

    === "CLI"

        الأوامر CLI متاحة لتشغيل النماذج مباشرة:

        ```bash
        # تحميل نموذج YOLOv8n المُدرب مسبقًا على COCO وتدريبه على مجموعة البيانات المثالية COCO8 لمدة 100 عصر
        yolo train model=yolov8n.pt data=coco8.yaml epochs=100 imgsz=640

        # تحميل نموذج YOLOv8n المُدرب مسبقًا على COCO وتشغيل الاستدلال على صورة 'bus.jpg'
        yolo predict model=yolov8n.pt source=path/to/bus.jpg
        ```

## المساهمة بنماذج جديدة

هل أنت مهتم بالمساهمة بنموذجك في Ultralytics؟ رائع! نحن دائمًا منفتحون على توسيع محفظة النماذج لدينا.

1. **احفظ نسخة عن المستودع**: ابدأ بحفظ نسخة عن [مستودع Ultralytics على GitHub](https://github.com/ultralytics/ultralytics).

2. **استنسخ نسختك**: انسخ نسختك إلى جهازك المحلي وأنشئ فرعًا جديدًا للعمل عليه.

3. **طبق نموذجك**: أضف نموذجك متبعًا معايير وإرشادات البرمجة الموفرة في دليل [المساهمة](../../help/contributing.md) لدينا.

4. **اختبر بدقة**: تأكد من اختبار نموذجك بشكل مكثف، سواء بشكل منفصل أو كجزء من المسار البرمجي.

5. **أنشئ Pull Request**: بمجرد أن تكون راضًيا عن نموذجك، قم بإنشاء طلب سحب إلى المستودع الرئيسي للمراجعة.

6. **مراجعة الكود والدمج**: بعد المراجعة، إذا كان نموذجك يلبي معاييرنا، سيتم دمجه في المستودع الرئيسي.

للخطوات التفصيلية، يرجى الرجوع إلى دليل [المساهمة](../../help/contributing.md).
