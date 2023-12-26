---
comments: true
description: استكشف أساليب مختلفة لتثبيت Ultralytics باستخدام pip و conda و git و Docker. تعرّف على كيفية استخدام Ultralytics مع واجهة سطر الأوامر أو ضمن مشاريع Python الخاصة بك.
keywords: تثبيت Ultralytics, pip install Ultralytics, Docker install Ultralytics, Ultralytics command line interface, Ultralytics Python interface
---

## تثبيت Ultralytics

يوفر Ultralytics طرق تثبيت مختلفة بما في ذلك pip و conda و Docker. يمكنك تثبيت YOLOv8 عن طريق حزمة `ultralytics` من خلال pip للإصدار الأحدث والمستقر أو من خلال استنساخ [مستودع Ultralytics على GitHub](https://github.com/ultralytics/ultralytics) للحصول على الإصدار الأحدث. يمكن استخدام Docker لتنفيذ الحزمة في حاوية معزولة، وتجنب التثبيت المحلي.

!!! Note "ملاحظة"

    🚧 تم بناء وثائقنا متعددة اللغات حاليًا، ونعمل بجد لتحسينها. شكرًا لك على صبرك! 🙏

!!! Example "تثبيت"

    === "تثبيت باستخدام pip (الموصَى به)"
        قم بتثبيت حزمة `ultralytics` باستخدام pip، أو قم بتحديث التثبيت الحالي عن طريق تشغيل `pip install -U ultralytics`. قم بزيارة مؤشر Python Package Index (PyPI) للحصول على مزيد من التفاصيل حول حزمة `ultralytics`: [https://pypi.org/project/ultralytics/](https://pypi.org/project/ultralytics/).

        [![نسخة PyPI](https://badge.fury.io/py/ultralytics.svg)](https://badge.fury.io/py/ultralytics) [![التنزيلات](https://static.pepy.tech/badge/ultralytics)](https://pepy.tech/project/ultralytics)

        ```bash
        # قم بتثبيت حزمة ultralytics من PyPI
        pip install ultralytics
        ```

        يمكنك أيضًا تثبيت حزمة `ultralytics` مباشرة من مستودع GitHub [repository](https://github.com/ultralytics/ultralytics). قد يكون ذلك مفيدًا إذا كنت ترغب في الحصول على الإصدار التجريبي الأحدث. تأكد من تثبيت أداة الأوامر Git على نظامك. يُثبّت الأمر `@main` الفرع `main` ويمكن تعديله إلى فرع آخر، على سبيل المثال `@my-branch`، أو يمكن إزالته تمامًا للانتقال إلى الفرع الرئيسي `main`.

        ```bash
        # قم بتثبيت حزمة ultralytics من GitHub
        pip install git+https://github.com/ultralytics/ultralytics.git@main
        ```


    === "تثبيت باستخدام conda"
        Conda هو مدير حزم بديل لـ pip ويمكن استخدامه أيضًا للتثبيت. قم بزيارة Anaconda للحصول على مزيد من التفاصيل على [https://anaconda.org/conda-forge/ultralytics](https://anaconda.org/conda-forge/ultralytics). يمكن العثور على مستودع Ultralytics feedstock لتحديث حزمة conda على [https://github.com/conda-forge/ultralytics-feedstock/](https://github.com/conda-forge/ultralytics-feedstock/).


        [![وصفة conda](https://img.shields.io/badge/recipe-ultralytics-green.svg)](https://anaconda.org/conda-forge/ultralytics) [![تنزيلات conda](https://img.shields.io/conda/dn/conda-forge/ultralytics.svg)](https://anaconda.org/conda-forge/ultralytics) [![إصدار conda](https://img.shields.io/conda/vn/conda-forge/ultralytics.svg)](https://anaconda.org/conda-forge/ultralytics) [![منصات conda](https://img.shields.io/conda/pn/conda-forge/ultralytics.svg)](https://anaconda.org/conda-forge/ultralytics)

        ```bash
        # قم بتثبيت حزمة ultralytics باستخدام conda
        conda install -c conda-forge ultralytics
        ```

        !!! Note "ملاحظة"

            إذا كنت تقوم بالتثبيت في بيئة CUDA، فإن الممارسة الجيدة هي تثبيت `ultralytics`, `pytorch` و `pytorch-cuda` في نفس الأمر للسماح لمدير حزم conda بحل أي تعارضات، أو وإلا فقوم بتثبيت  `pytorch-cuda` في نهاية الأمر للسماح له بتجاوز حزمة `pytorch` المحددة لوحدة المعالجة المركزية إذا لزم الأمر.
            ```bash
            # قم بتثبيت كافة الحزم معًا باستخدام conda
            conda install -c pytorch -c nvidia -c conda-forge pytorch torchvision pytorch-cuda=11.8 ultralytics
            ```

        ### صورة Docker في Conda

        تتوفر أيضًا صور Docker لـ Conda لـ Ultralytics من [DockerHub](https://hub.docker.com/r/ultralytics/ultralytics). تستند هذه الصور إلى [Miniconda3](https://docs.conda.io/projects/miniconda/en/latest/) وهي وسيلة بسيطة لبدء استخدام `ultralytics` في بيئة Conda.

        ```bash
        # قم بتعيين اسم الصورة بوصفه متغير
        t=ultralytics/ultralytics:latest-conda

        # اسحب أحدث صورة ultralytics من Docker Hub
        sudo docker pull $t

        # قم بتشغيل صورة ultralytics في حاوية مع دعم GPU
        sudo docker run -it --ipc=host --gpus all $t  # all GPUs
        sudo docker run -it --ipc=host --gpus '"device=2,3"' $t  #  قد يتم تحديد GPUs
        ```

    === "استنساخ Git"
        قم بنسخ مستودع `ultralytics` إذا كنت مهتمًا بالمساهمة في التطوير أو ترغب في تجربة الشفرة المصدرية الأحدث. بعد الاستنساخ، انتقل إلى الدليل وقم بتثبيت الحزمة في وضع التحرير `-e` باستخدام pip.
        ```bash
        # قم بنسخ مستودع ultralytics
        git clone https://github.com/ultralytics/ultralytics

        # انتقل إلى الدليل المنسوخ
        cd ultralytics

        # قم بتثبيت الحزمة في وضع التحرير
        pip install -e .
        ```

    === "Docker"

         تمكنك من استخدام Docker بسهولة لتنفيذ حزمة `ultralytics` في حاوية معزولة، مما يضمن أداءً سلسًا ومتسقًا في مختلف البيئات. عن طريق اختيار إحدى صور Docker الأصلية لـ `ultralytics` من [Docker Hub](https://hub.docker.com/r/ultralytics/ultralytics)، لن تتجنب فقط تعقيد التثبيت المحلي ولكنك ستستفيد أيضًا من وصول إلى بيئة عمل متحققة وفعالة. يقدم Ultralytics 5 صور Docker مدعومة رئيسية، يتم تصميم كل منها لتوفير توافق عالي وكفاءة لمنصات وحالات استخدام مختلفة:

        <a href="https://hub.docker.com/r/ultralytics/ultralytics"><img src="https://img.shields.io/docker/pulls/ultralytics/ultralytics?logo=docker" alt="Docker Pulls"></a>

        - **Dockerfile:** صورة GPU الموصى بها للتدريب.
        - **Dockerfile-arm64:** محسّن لبنية ARM64، مما يتيح النشر على أجهزة مثل Raspberry Pi ومنصات أخرى تعتمد على ARM64.
        - **Dockerfile-cpu:** إصدار مناسب للتحكم بوحدة المعالجة المركزية فقط بدون دعم لل GPU.
        - **Dockerfile-jetson:** مصمم خصيصًا لأجهزة NVIDIA Jetson، ويدمج دعمًا لل GPU المحسن لهذه المنصات.
        - **Dockerfile-python:** صورة صغيرة بها فقط Python والتبعيات الضرورية، مثالية للتطبيقات والتطوير الخفيف.
        - **Dockerfile-conda:** قائمة على Miniconda3 مع تثبيت conda لحزمة ultralytics.

        فيما يلي الأوامر للحصول على أحدث صورة وتشغيلها:

        ```bash
        # قم بتعيين اسم الصورة بوصفه متغير
        t=ultralytics/ultralytics:latest

        # اسحب أحدث صورة ultralytics من Docker Hub
        sudo docker pull $t

        # قم بتشغيل صورة ultralytics في حاوية مع دعم GPU
        sudo docker run -it --ipc=host --gpus all $t  # all GPUs
        sudo docker run -it --ipc=host --gpus '"device=2,3"' $t  #  قد يتم تحديد GPUs
        ```

        يقوم الأمر أعلاه بتهيئة حاوية Docker بأحدث صورة `ultralytics`. يُسند العلامة `-it` جهازًا افتراضيًا TTY ويحافظ على فتح stdin لتمكينك من التفاعل مع الحاوية. تعيين العلامة `--ipc=host` مساحة اسم IPC (Inter-Process Communication) إلى المضيف، وهو أمر ضروري لمشاركة الذاكرة بين العمليات. تُمكّن العلامة `--gpus all` الوصول إلى كل وحدات المعالجة المركزية الرسومية المتاحة داخل الحاوية، مما هو أمر حاسم للمهام التي تتطلب حسابات GPU.

        ملاحظة: للعمل مع الملفات على جهازك المحلي داخل الحاوية، استخدم مجلدات Docker لتوصيل دليل محلي بالحاوية:

        ```bash
        # مجلد الدليل المحلي بالحاوية
        sudo docker run -it --ipc=host --gpus all -v /path/on/host:/path/in/container $t
        ```

        قم بتغيير `/path/on/host` بمسار الدليل على جهازك المحلي، و `/path/in/container` باالمسار المطلوب داخل حاوية Docker للوصول إليه.

        للاستفادة القصوى من استخدام Docker المتقدم، لا تتردد في استكشاف [دليل Ultralytics Docker](https://docs.ultralytics.com/guides/docker-quickstart/).

راجع ملف `requirements.txt` الخاص بـ `ultralytics` [هنا](https://github.com/ultralytics/ultralytics/blob/main/requirements.txt) للحصول على قائمة المتطلبات. يُرجى ملاحظة أن جميع الأمثلة أعلاه يتم تثبيت جميع المتطلبات المطلوبة.

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/_a7cVL9hqnk"
    title="YouTube player" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>شاهد:</strong> دليل فتع Ultralytics YOLO السريع
</p>

!!! Tip "نصيحة"

    يختلف متطلبات PyTorch حسب نظام التشغيل ومتطلبات CUDA، لذا يُوصَى بتثبيت PyTorch أولاً باستخدام التعليمات الموجودة في [https://pytorch.org/get-started/locally](https://pytorch.org/get-started/locally).

    <a href="https://pytorch.org/get-started/locally/">
        <img width="800" alt="PyTorch تعليمات التثبيت" src="https://user-images.githubusercontent.com/26833433/228650108-ab0ec98a-b328-4f40-a40d-95355e8a84e3.png">
    </a>

## استخدم Ultralytics مع واجهة سطر الأوامر (CLI)

تتيح واجهة سطر الأوامر (CLI) في Ultralytics تشغيل أوامر بسيطة بدون الحاجة إلى بيئة Python. لا تحتاج CLI إلى أي تخصيص أو كود Python. يمكنك ببساطة تشغيل جميع المهام من الطرفية باستخدام الأمر `yolo`. تحقق من [دليل CLI](/../usage/cli.md) لمعرفة المزيد حول استخدام YOLOv8 من سطر الأوامر.

!!! Example "مثال"

    === "الصيغة"
        تستخدم أوامر Ultralytics `yolo` الصيغة التالية:
        ```bash
        yolo TASK MODE ARGS
        ```

        - `TASK` (اختياري) أحد التالي ([detect](tasks/detect.md), [segment](tasks/segment.md), [classify](tasks/classify.md), [pose](tasks/pose.md))
        - `MODE` (مطلوب) واحد من ([train](modes/train.md), [val](modes/val.md), [predict](modes/predict.md), [export](modes/export.md), [track](modes/track.md))
        - `ARGS` (اختياري) أزواج "arg=value" مثل `imgsz=640` التي تستبدل القيم الافتراضية.

        راجع جميع `ARGS` [هنا](/../usage/cfg.md) أو باستخدام الأمر `yolo cfg` في سطر الأوامر.

    === "التدريب"
        قم بتدريب نموذج اكتشاف لمدة 10 حلقات مع سعر تعلم بدءي 0.01
        ```bash
        yolo train data=coco128.yaml model=yolov8n.pt epochs=10 lr0=0.01
        ```

    === "التنبؤ"
        تنبؤ بفيديو YouTube باستخدام نموذج تجزئة معتمد مسبقًا عند حجم الصورة 320:
        ```bash
        yolo predict model=yolov8n-seg.pt source='https://youtu.be/LNwODJXcvt4' imgsz=320
        ```

    === "التحقق"
        التحقق من نموذج اكتشاف معتمد مسبقًا على دُفعَة واحدة وحجم صورة قدره 640:
        ```bash
        yolo val model=yolov8n.pt data=coco128.yaml batch=1 imgsz=640
        ```

    === "التصدير"
        قم بتصدير نموذج فئة YOLOv8n إلى تنسيق ONNX على حجم صورة 224 بواسطة 128 (لا يلزم TASK)
        ```bash
        yolo export model=yolov8n-cls.pt format=onnx imgsz=224,128
        ```

    === "خاص"
        قم بتشغيل أوامر خاصة لعرض الإصدارة وعرض الإعدادات وتشغيل عمليات التحقق والمزيد:
        ```bash
        yolo help
        yolo checks
        yolo version
        yolo settings
        yolo copy-cfg
        yolo cfg
        ```

!!! Warning "تحذير"
يجب تمرير الوسوم كأزواج "arg=val"، وأن تُفصل بعلامة تساوي `=` وأن تُفصل بمسافات بين الأزواج. لا تستخدم بادئات الوسوم `--` أو فواصل `,` بين الوسوم.

    - `yolo predict model=yolov8n.pt imgsz=640 conf=0.25`  ✅
    - `yolo predict model yolov8n.pt imgsz 640 conf 0.25`  ❌  (مفقود العلامة المساواة)
    - `yolo predict model=yolov8n.pt, imgsz=640, conf=0.25`  ❌ (لا تستخدم `,`)
    - `yolo predict --model yolov8n.pt --imgsz 640 --conf 0.25`  ❌ (لا تستخدم `--`)

[دليل CLI](/../usage/cli.md){ .md-button }

## استخدم Ultralytics مع Python

تسمح واجهة Python في YOLOv8 بالتكامل السلس في مشاريع Python الخاصة بك، مما يجعل من السهل تحميل النموذج وتشغيله ومعالجة نتائجه. المصممة ببساطة وسهولة الاستخدام في الاعتبار، تمكن واجهة Python المستخدمين من تنفيذ الكشف على الكائنات والتجزئة والتصنيف في مشاريعهم. يجعل هذا واجهة YOLOv8 Python أداة قيمة لأي شخص يرغب في دمج هذه الوظائف في مشاريعهم باسياتو.

على سبيل المثال، يمكن للمستخدمين تحميل نموذج، تدريبه، تقييم أدائه على مجموعة التحقق، وحتى تصديره إلى تنسيق ONNX ببضعة أسطر فقط من الشفرة. تحقق من [دليل Python](/../usage/python.md) لمعرفة المزيد حول استخدام YOLOv8 داخل مشاريعك الخاصة.

!!! Example "مثال"

    ```python
    from ultralytics import YOLO

    # أنشئ نموذج YOLO جديد من البداية
    model = YOLO('yolov8n.yaml')

    # قم بتحميل نموذج YOLO معتمد مسبقًا (موصَى به للتدريب)
    model = YOLO('yolov8n.pt')

    # قم بتدريب النموذج باستخدام مجموعة البيانات 'coco128.yaml' لمدة 3 حلقات
    results = model.train(data='coco128.yaml', epochs=3)

    # قم بتقييم أداء النموذج على مجموعة التحقق
    results = model.val()

    # قم بإجراء الكشف على صورة باستخدام النموذج
    results = model('https://ultralytics.com/images/bus.jpg')

    # قم بتصدير النموذج إلى تنسيق ONNX
    success = model.export(format='onnx')
    ```

[دليل Python](/../usage/python.md){.md-button .md-button--primary}

## إعدادات Ultralytics

يوفر مكتبة Ultralytics نظامًا قويًا لإدارة الإعدادات لتمكين التحكم بمحاكاة تفصيلية لتجاربك. من خلال استخدام `SettingsManager` في الوحدة `ultralytics.utils`، يمكن للمستخدمين الوصول بسهولة إلى إعداداتهم وتعديلها. يتم تخزينها في ملف YAML ويمكن عرضها أو تعديلها إما مباشرة في بيئة Python أو من خلال واجهة سطر الأوامر (CLI).

### فحص الإعدادات

للحصول على فهم للتكوين الحالي لإعداداتك، يمكنك عرضها مباشرةً:

!!! Example "عرض الإعدادات"

    === "Python"
        يُمكنك استخدام Python لعرض الإعدادات الخاصة بك. ابدأ بـاستيراد الكائن `settings` من وحدة `ultralytics`. استخدم الأوامر التالية لطباعة الإعدادات والعودة منها:
        ```python
        from ultralytics import settings

        # عرض كل الإعدادات
        print(settings)

        # إرجاع إعداد محدد
        value = settings['runs_dir']
        ```

    === "CLI"
        بدلاً من ذلك، واجهة سطر الأوامر تسمح لك بالتحقق من الإعدادات الخاصة بك باستخدام أمر بسيط:
        ```bash
        yolo settings
        ```

### تعديل الإعدادات

يسمح لك Ultralytics بتعديل الإعدادات بسهولة. يمكن تنفيذ التغييرات بالطرق التالية:

!!! Example "تحديث الإعدادات"

    === "Python"
        داخل بيئة Python، اطلب الطريقة `update` على الكائن `settings` لتغيير إعداداتك:

        ```python
        from ultralytics import settings

        # تحديث إعداد واحد
        settings.update({'runs_dir': '/path/to/runs'})

        # تحديث إعدادات متعددة
        settings.update({'runs_dir': '/path/to/runs', 'tensorboard': False})

        # إعادة الإعدادات إلى القيم الافتراضية
        settings.reset()
        ```

    === "CLI"
        إذا كنت تفضل استخدام واجهة سطر الأوامر، يمكنك استخدام الأوامر التالية لتعديل إعداداتك:

        ```bash
        # تحديث إعداد واحد
        yolo settings runs_dir='/path/to/runs'

        # تحديث إعدادات متعددة
        yolo settings runs_dir='/path/to/runs' tensorboard=False

        # إعادة الإعدادات إلى القيم الافتراضية
        yolo settings reset
        ```

### فهم الإعدادات

يوفر الجدول أدناه نظرة عامة على الإعدادات المتاحة للضبط في Ultralytics. يتم توضيح كل إعداد بالإضافة إلى قيمة مثالية ونوع البيانات ووصف موجز.

| الاسم              | القيمة المثالية       | نوع البيانات | الوصف                                                                                                       |
|--------------------|-----------------------|--------------|-------------------------------------------------------------------------------------------------------------|
| `settings_version` | `'0.0.4'`             | `str`        | إصدار إعدادات Ultralytics (مختلف عن إصدار Ultralytics [pip](https://pypi.org/project/ultralytics/))         |
| `datasets_dir`     | `'/path/to/datasets'` | `str`        | المسار الذي يتم تخزينه فيه مجموعات البيانات                                                                 |
| `weights_dir`      | `'/path/to/weights'`  | `str`        | المسار الذي يتم تخزينه فيه أوزان النموذج                                                                    |
| `runs_dir`         | `'/path/to/runs'`     | `str`        | المسار الذي يتم تخزينه فيه تشغيل التجارب                                                                    |
| `uuid`             | `'a1b2c3d4'`          | `str`        | مُعرِّف فريد لإعدادات الحالية                                                                               |
| `sync`             | `True`                | `bool`       | ما إذا كان يتم مزامنة التحليلات وحوادث الأعطال إلى HUB                                                      |
| `api_key`          | `''`                  | `str`        | HUB الخاص بـ Ultralytics [API Key](https://hub.ultralytics.com/settings?tab=api+keys)                       |
| `clearml`          | `True`                | `bool`       | ما إذا كان يتم استخدام ClearML لتسجيل التجارب                                                               |
| `comet`            | `True`                | `bool`       | ما إذا كان يتم استخدام [Comet ML](https://bit.ly/yolov8-readme-comet) لتتبع وتصور التجارب                   |
| `dvc`              | `True`                | `bool`       | ما إذا كان يتم استخدام [DVC لتتبع التجارب](https://dvc.org/doc/dvclive/ml-frameworks/yolo) والتحكم في النسخ |
| `hub`              | `True`                | `bool`       | ما إذا كان يتم استخدام [Ultralytics HUB](https://hub.ultralytics.com) للتكامل                               |
| `mlflow`           | `True`                | `bool`       | ما إذا كان يتم استخدام MLFlow لتتبع التجارب                                                                 |
| `neptune`          | `True`                | `bool`       | ما إذا كان يتم استخدام Neptune لتتبع التجارب                                                                |
| `raytune`          | `True`                | `bool`       | ما إذا كان يتم استخدام Ray Tune لضبط الحساسية                                                               |
| `tensorboard`      | `True`                | `bool`       | ما إذا كان يتم استخدام TensorBoard للتصور                                                                   |
| `wandb`            | `True`                | `bool`       | ما إذا كان يتم استخدام Weights & Biases لتسجيل البيانات                                                     |

أثناء تنقلك في مشاريعك أو تجاربك، تأكد من مراجعة هذه الإعدادات لضمان تكوينها بشكل مثالي وفقًا لاحتياجاتك.
