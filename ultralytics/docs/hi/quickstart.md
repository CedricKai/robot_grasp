---
comments: true
description: Ultralytics को स्थापित करने के विभिन्न तरीकों के बारे में जानें। Ultralytics को pip, conda, git और Docker का उपयोग करके स्थापित करें। Ultralytics का उपयोग कमांड लाइन इंटरफेस या अपनी Python परियोजनाओं के भीतर करना सीखें।
keywords: Ultralytics स्थापना, pip install Ultralytics, Docker install Ultralytics, Ultralytics कमांड लाइन इंटरफेस, Ultralytics Python इंटरफेस
---

## Ultralytics स्थापित करें

Ultralytics ने pip, conda और Docker सहित कई स्थापना विधियाँ प्रदान की हैं। नवीनतम स्थिर संस्करण के लिए `ultralytics` pip पैकेज का उपयोग करके YOLOv8 स्थापित करें या सबसे अद्यतित संस्करण के लिए [Ultralytics GitHub repository](https://github.com/ultralytics/ultralytics) क्लोन करें। Docker का उपयोग करके, स्थानीय स्थापना से बच कर, एक छोटे जगह में पैकेज के नए संस्करण का निष्पादन किया जा सकता है।

!!! Note "नोट"

    🚧 हमारे बहुभाषीय दस्तावेज़ीकरण की वर्तमान में निर्माणाधीन है और हम उसे सुधारने के लिए कठिनताओं पर काम कर रहे हैं। आपके धैर्य के लिए धन्यवाद! 🙏

!!! Example "स्थापित करें"

    === "Pip स्थापित करें (अनुशंसित)"
        यदि आपके पास पिछले संस्करण का स्थापना है, तो पिप का उपयोग करके `ultralytics` पैकेज को स्थापित करने के लिए `pip install -U ultralytics` कमांड चलाएं। `ultralytics` पैकेज के बारे में अधिक विवरण के लिए [Python Package Index (PyPI)](https://pypi.org/project/ultralytics/) पर जाएं।

        [![PyPI version](https://badge.fury.io/py/ultralytics.svg)](https://badge.fury.io/py/ultralytics) [![Downloads](https://static.pepy.tech/badge/ultralytics)](https://pepy.tech/project/ultralytics)

        ```bash
        # PyPI से ultralytics पैकेज का स्थापना करें
        pip install ultralytics
        ```

        आप इसे सीधे [GitHub repository](https://github.com/ultralytics/ultralytics) से भी स्थापित कर सकते हैं। यह अद्यतन संस्करण प्राप्त करना चाहते हैं तो यह सर्वोत्तम हो सकता है। इसके लिए अपने सिस्टम पर गिट कमांड-लाइन टूल स्थापित होना चाहिए। `@main` अपदेश की `main` शाखा को स्थापित करता है और इसे दूसरी शाखा, उदा. `@my-branch`, में संशोधित किया जा सकता है, या पूर्णतः हटा दिया जा सकता है, ताकि यह डिफ़ॉल्ट रूप से `main` शाखा को ले जाए।

        ```bash
        # GitHub से ultralytics पैकेज का स्थापना करें
        pip install git+https://github.com/ultralytics/ultralytics.git@main
        ```


    === "Conda स्थापित करें"
        स्थापना के लिए pip के बदले Conda एक वैकल्पिक पैकेज प्रबंधक है जिसे आप स्थापना के लिए उपयोग कर सकते हैं। किसी भी जानकारी के लिए [Anaconda की मुख्य साइट](https://anaconda.org/conda-forge/ultralytics) पर जाएं। कंडा पैकेज की अद्यतन और संसाधन रिपो के लिए [यहां](https://github.com/conda-forge/ultralytics-feedstock/) देखें।


        [![Conda Recipe](https://img.shields.io/badge/recipe-ultralytics-green.svg)](https://anaconda.org/conda-forge/ultralytics) [![Conda Downloads](https://img.shields.io/conda/dn/conda-forge/ultralytics.svg)](https://anaconda.org/conda-forge/ultralytics) [![Conda Version](https://img.shields.io/conda/vn/conda-forge/ultralytics.svg)](https://anaconda.org/conda-forge/ultralytics) [![Conda Platforms](https://img.shields.io/conda/pn/conda-forge/ultralytics.svg)](https://anaconda.org/conda-forge/ultralytics)

        ```bash
        # conda का उपयोग करके ultralytics पैकेज का स्थापना करें
        conda install -c conda-forge ultralytics
        ```

        !!! Note "नोट"

            यदि आप CUDA परिवेश में स्थापित कर रहे हैं तो सर्वोत्तम अनुशंसा है कि आप कमांड-लाइन पर `pytorch` और `pytorch-cuda` स्थापित करने के लिए कमांड एक साथ इंस्टॉल करें ताकि कोण्डा पैकेज प्रबंधक को कोई भी टकराव सुलझाने के लिए अनुमति मिले, या फिर जरूरत पड़ने पर CPU-विशिष्ट `pytorch` पैकेज को CPU-विशिष्ट होने वाले `pytorch-cuda` पैकेज को अधिरोहित करने की अनुमति दें।
            ```bash
            # conda का उपयोग करके सभी पैकेजों को एक साथ स्थापित करें
            conda install -c pytorch -c nvidia -c conda-forge pytorch torchvision pytorch-cuda=11.8 ultralytics
            ```

        ### Conda Docker इमेज

        Ultralytics Conda Docker इमेज [DockerHub](https://hub.docker.com/r/ultralytics/ultralytics) से उपलब्ध हैं। ये इमेजेज [Miniconda3](https://docs.conda.io/projects/miniconda/en/latest/) पर आधारित हैं और `ultralytics` का उपयोग Conda पर्यावरण में करने के लिए एक सरल तरीका है।

        ```bash
        # रूपरेखा नाम को एक चर के रूप में सेट करें
        t=ultralytics/ultralytics:latest-conda

        # Docker Hub से नवीनतम ultralytics इमेज को पुल करें
        sudo docker pull $t

        # जीपीयू समर्थन वाले कंटेनर में ultralytics इमेज चलाएं
        sudo docker run -it --ipc=host --gpus all $t  # सभी जीपीयू
        sudo docker run -it --ipc=host --gpus '"device=2,3"' $t  # जीपीयू द्वारा निर्दिष्ट करें
        ```

    === "Git क्लोन"
        यदि आप विकास में योगदान करने में रुचि रखते हैं या नवीनतम स्रोत कोड के साथ प्रयोग करने की इच्छा रखते हैं, तो `ultralytics` रिपॉजिटरी क्लोन करें। क्लोनिंग के बाद, उस निर्दिष्टित संदर्भ में नेविगेट करें और पैकेज को पहचानने के लिए pip का उपयोग करते हुए संगठनात्मक मोड `-e` के साथ पैकेज स्थापित करें।
        ```bash
        # ultralytics रिपॉजिटरी क्लोन करें
        git clone https://github.com/ultralytics/ultralytics

        # क्लोन की गई निर्देशिका में नेविगेट करें
        cd ultralytics

        # विकास के लिए संगठनात्मक मोड में पैकेज स्थापित करें
        pip install -e .
        ```

    === "Docker"

        Docker का उपयोग करके `ultralytics` पैकेज का आसानी से निष्पादन करें और इसे रखरखाव में बेहद सुगम बनाएं, इस पैकेज का उपयोग करें, विभिन्न पर्यावरणों पर सतत और सुगम प्रदर्शन सुनिश्चित करने के लिए। [Docker Hub](https://hub.docker.com/r/ultralytics/ultralytics) से सत्यापित कार्यकारी वातावरण तक पहुंच के लिए Ultralytics 5 मुख्य समर्थित Docker इमेज उपलब्ध हैं, जो विभिन्न प्लेटफ़ॉर्म और उपयोग मामलों के लिए उच्च संगतता और प्रदार्थशीलता प्रदान करने के लिए डिज़ाइन किए गए हैं:

        <a href="https://hub.docker.com/r/ultralytics/ultralytics"><img src="https://img.shields.io/docker/pulls/ultralytics/ultralytics?logo=docker" alt="डॉकर पुल्ल्स"></a>

        - **Dockerfile:** प्रशिक्षण के लिए अद्यतन संस्करण के लिए अनुशंसित GPU चित्र।
        - **Dockerfile-arm64:** ARM64 वाणिज्यिकरण के लिए अनुकूलित, Raspberry Pi और अन्य ARM64 आधारित प्लेटफ़ॉर्म पर यातायात की अनुमति देता है।
        - **Dockerfile-cpu:** GPU रहित पतला मॉडल, उबंटू आधारित योग्यता तक पुनर्निर्माण के लिए उपयुक्त है।
        - **Dockerfile-jetson:** NVIDIA Jetson उपकरणों के लिए आदर्शों के आधार पर गीयू समर्थन मिलान, इन प्लेटफ़ॉर्मों के लिए अनुकूल यूपीयू समर्थन समेकित करता है।
        - **Dockerfile-python:** केवल Python और आवश्यकता प्रतिस्थापित करने वाले न्यूनतम छवि, हल्के ऐप्स और विकास के लिए आदर्श छवि।
        - **Dockerfile-conda:**  Miniconda3 पर आधारित, Ultralytics पैकेज के कोण्डा स्थापना के साथ।

        निम्नलिखित कमांडों का उपयोग करके नवीनतम छवि लाएँ और उसे निष्पादित करें:

        ```bash
        # छवि नाम को एक चर के रूप में सेट करें
        t=ultralytics/ultralytics:latest

        # Docker Hub से नवीनतम ultralytics छवि पुल करें
        sudo docker pull $t

        # जीपीयू समर्थन वाले कंटेनर में ultralytics छवि चलाएं
        sudo docker run -it --ipc=host --gpus all $t  # सभी जीपीयू
        sudo docker run -it --ipc=host --gpus '"device=2,3"' $t  # जीपीयू द्वारा निर्दिष्ट करें
        ```

        उपरोक्त कमांड ने एक Docker कंटेनर को एक्सेस करने के लिए उत्थान किया है। `-it` झंझटी एक प्रतीक TTY को निर्धारित करती है और stdin खुली रखती है, जिससे आप कंटेनर के साथ इंटरैक्ट कर सकते हैं। `--ipc=host` झंझटी IPC (Inter-Process Communication) नेमस्पेस को होस्ट पर सेट करता है, जो प्रक्रियाओं के बीच मेमोरी साझा करने के लिए आवश्यक होता है। `--gpus all` निर्दिष्ट जीपीयू कंटेनर के बीतर सभी उपलब्ध जीपीयू के लिए पहुंच सक्षम करता है, जो जीपीयू हस्तक्षेप आवश्यकता वाले कार्यों के लिए महत्वपूर्ण है।

        ध्यान दें: कंटेनर में स्थिति में अपनी स्थानीय मशीन पर फ़ाइलों के साथ काम करने के लिए Docker वॉल्यूम का उपयोग करें:

        ```bash
        # स्थानीय निर्देशिका को कंटेनर में निर्देशिका में माउंट करें
        sudo docker run -it --ipc=host --gpus all -v /path/on/host:/path/in/container $t
        ```

        `/path/on/host` को अपनी स्थानीय मशीन पर निर्देशिका पथ के साथ बदलें और `/path/in/container` को कंटेनर में योग्यता तक पथ बदलें जिससे पहुंच मिल सके।

        पूर्ण Docker उपयोग के लिए, आप [Ultralytics Docker मार्गदर्शिका](https://docs.ultralytics.com/guides/docker-quickstart/) के अन्वेषण कर सकते हैं।

`ultralytics` के लिए सभी आवश्यकताओं की सूची के लिए `ultralytics` [requirements.txt](https://github.com/ultralytics/ultralytics/blob/main/requirements.txt) फ़ाइल देखें। ध्यान दें कि उपरोक्त सभी उदाहरणों में सभी आवश्यकताएं स्थापित होती हैं।

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/_a7cVL9hqnk"
    title="YouTube वीडियो प्लेयर" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>देखें:</strong> Ultralytics YOLO Quick Start Guide
</p>

!!! Tip "सुझाव"

    ऑपरेटिंग सिस्टम और CUDA आवश्यकताओं के अनुसार PyTorch आवश्यकताएं अलग-अलग हो सकती हैं, इसलिए अनुशंसा की जाती है कि पहले PyTorch स्थापित करने के लिए इंस्ट्रक्शंस पर जाएं। [https://pytorch.org/get-started/locally](https://pytorch.org/get-started/locally) पर उपयोग के बारे में अधिक जानकारी के लिए।

    <a href="https://pytorch.org/get-started/locally/">
        <img width="800" alt="PyTorch Installation Instructions" src="https://user-images.githubusercontent.com/26833433/228650108-ab0ec98a-b328-4f40-a40d-95355e8a84e3.png">
    </a>

## CLI के साथ Ultralytics का उपयोग करें

Ultralytics कमांड लाइन इंटरफ़ेस (CLI) आसान एकल-पंक्ति कमांड के लिए संक्षेप में होसला अद्यतित करता है, पायथन पर्यावरण की ज़रूरत के बिना। CLI कोई अनुकूलन या पायथन कोड की आवश्यकता नहीं होती है। आप केवल `yolo` कमांड के साथ टर्मिनल से सभी कार्यों को चला सकते हैं। CLI से YOLOv8 का उपयोग करने के बारे में और अधिक जानने के लिए [CLI Guide](/../usage/cli.md) देखें।

!!! Example "उदाहरण"

    === "संयोजन"
        Ultralytics `yolo` कमांड का उपयोग निम्नलिखित प्रारूप का उपयोग करता है:
        ```bash
        yolo टास्क मोड ARGS
        ```

        - `टास्क` (वैकल्पिक) इनमें से एक है ([खोजो](tasks/detect.md), [खंड](tasks/segment.md), [वर्गीकरण करो](tasks/classify.md), [स्थिति](tasks/pose.md))
        - `मोड` (आवश्यक) इनमें से एक है ([प्रशिक्षण](modes/train.md), [मान्य](modes/val.md), [पूर्वानुमान](modes/predict.md), [निर्यात](modes/export.md), [ट्रैक](modes/track.md))
        - `ARGS` (वैकल्पिक) `imgsz=640` जैसे `arg=मान` जो डिफ़ॉल्ट को ओवरराइड करते हैं।

        सभी `ARGS` को पूर्ण [Configuration Guide](/../usage/cfg.md) या `yolo cfg` CLI कमांड के साथ देखें।

    === "प्रशिक्षण"
        प्रारंभिक शिक्षण और language के साथ 10 एपोक्स के लिए एक डिटेक्शन मॉडल प्रशिक्षित करें, जहां
        इंगिती शिक्षण दर 0.01 है
        ```bash
        yolo ट्रेन data=coco128.yaml model=yolov8n.pt epochs=10 lr0=0.01
        ```

    === "पूर्वानुमान"
        पूर्व-प्रशिक्षित सेगमेंटेशन मॉडल का उपयोग करके YouTube वीडियो की भविष्यवाणी करें
        छवि आकार 320:
        ```bash
        yolo पूर्वानुमान model=yolov8n-seg.pt स्रोत='https://youtu.be/LNwODJXcvt4' imgsz=320
        ```

    === "मान्य करो"
        एक पूर्व-प्रशिक्षित डिटेक्शन मॉडल की मान्यता वाली प्रमाणित करें और इमेज का आकार 640 के बैच-आकार 1 के साथ देखें:
        ```bash
        yolo मान्य model=yolov8n.pt data=coco128.yaml batch=1 imgsz=640
        ```

    === "निर्यात करें"
        एक YOLOv8n वर्गीकरण मॉडल को ONNX प्रारूप में निर्यात करें, 224x224 के आकार पर छवि (कोई टास्क आवश्यक नहीं है)
        ```bash
        yolo निर्यात model=yolov8n-cls.pt format=onnx imgsz=224,128
        ```

    === "विशेष"
        संस्पेष्ट कमांडों को चलाएं ताकि संस्करण, सेटिंग देखें, चेक करें और अधिक देखें:
        ```bash
        yolo help
        yolo checks
        yolo version
        yolo settings
        yolo copy-cfg
        yolo cfg
        ```

!!! Warning "चेतावनी"

    ताकि दुविधा न हो, तज़्ज़ा सेटिंग को `arg=val` जोड़े के रूप में पार करना होगा, जिन्हें `=` रेखा द्वारा विभाजित किया जाता है और जोड़ों के बीच अंतरित होता है। `--` तर्क-पूर्वक अंटीरे शब्द या `,` अंतराल द्वारा तर्कों का उपयोग न करें।

    - `yolo predict model=yolov8n.pt imgsz=640 conf=0.25`  ✅
    - `yolo predict model yolov8n.pt imgsz 640 conf 0.25`  ❌ (अभाव `=`)
    - `yolo predict model=yolov8n.pt, imgsz=640, conf=0.25`  ❌ (`,` उपयोग न करें)
    - `yolo predict --model yolov8n.pt --imgsz 640 --conf 0.25`  ❌ (`--` उपयोग न करें)

एकेन्द्रीय योग्यताएँ [Configuration Guide](/../usage/cfg.md) या `yolo cfg` CLI कमांड के साथ देखें।

## Python के साथ Ultralytics का उपयोग करें

YOLOv8 का Python इंटरफ़ेस आपकी Python परियोजनाओं में अंकित मिलने के लिए एक आसान तकनीक प्रदान करता है, जिसे हमारे पास शामिल करना आसान हो जाता है। उपयोगकर्ताओं को उनके परियोजनाओं में आपातकालीन पहुंच, चलाने और मॉडल के आउटपुट की प्रसंस्करण करने की आसानी के साथ प्रश्नोत्तरी, खंड, और वर्गीकरण कार्यों के लिए सुविधाजनक मूल्य प्रदान करता है। इस तकनीक के साथ, उपयोगकर्ताओं के लिए यह अद्वितीय साधन है जो अपनी Python परियोजनाओं में इन गुणों को शामिल करने की इच्छा रखते हैं।

उदाहरण के लिए, उपयोगकर्ता संख्या गिनती के लिए कुछ-कुछ तारणी की योजना में मॉडल को लोड करके उसे प्रशिक्षित कर सकते हैं, इसका मूल्यांकन समाप्त कर सकते हैं और यदि आवश्यक हो, उसे ONNX प्रारूप में निर्यात कर सकते हैं। अपनी Python परियोजनाओं में YOLOv8 का उपयोग करने के बारे में और अधिक जानने के लिए [Python Guide](/../usage/python.md) देखें।

!!! Example "उदाहरण"

    ```python
    from ultralytics import YOLO

    # पूरी नई YOLO मॉडल बनाएँ
    model = YOLO('yolov8n.yaml')

    # प्रशिक्षित YOLO मॉडल लोड करें (प्रशिक्षण के लिए अनुशंसित है)
    model = YOLO('yolov8n.pt')

    # 3 एपोक्स के लिए "coco128.yaml" डेटासेट का उपयोग करके मॉडल को प्रशिक्षित करें
    results = model.train(data='coco128.yaml', epochs=3)

    # मॉडल के द्वारा मान्यता वाले सेट पर प्रदर्शन करें
    results = model.val()

    # मॉडल को उपयोग करके छवि पर डिटेक्शन करें
    results = model('https://ultralytics.com/images/bus.jpg')

    # मॉडल को ONNX प्रारूप में निर्यात करें
    success = model.export(format='onnx')
    ```

[Python Guide](/../usage/python.md){.md-button .md-button--primary}

## Ultralytics सेटिंग्स

Ultralytics लाइब्रेरी सेटिंग्स प्रबंधन प्रणाली प्रदान करती है ताकि आप अपने प्रयोगों पर फाइन-ग्रेन्ड नियंत्रण बनाए रख सकें। `ultralytics.utils` में स्थित `SettingsManager` का उपयोग करके उपयोगकर्ता अपनी सेटिंग्स तक पहुंच करके उन्हें पढ़ और बदल सकते हैं। इन्हें पायथन पर्यावरण के भीतर सीधे देखने और संशोधित करने के लिए, या CLI (कमांड लाइन इंटरफ़ेस) के माध्यम से किया जा सकता है।

### सेटिंग्स का गणना

अपनी सेटिंग्स के मौजूदा विन्यास की ओरदारी करने के लिए आप उन्हें सीधे देख सकते हैं:

!!! Example "सेटिंग्स देखें"

    === "पायथन"
        आप PyTorch से `ultralytics` मॉड्यूल में `सेटिंग्स` ऑब्जेक्ट को आयात करके अपनी सेटिंग्स देख सकते हैं। `settings` ऑब्जेक्ट पर प्रिंट और रिटर्न सेटिंग्स के लिए निम्नलिखित कमांडों का उपयोग करें:
        ```python
        from ultralytics import settings

        # सभी सेटिंग्स देखें
        print(settings)

        # एक विशेष सेटिंग प्राप्त करें
        value = settings['runs_dir']
        ```

    === "CLI"
        यदि आप प्राथमिकताएँ लेते हैं CLI का उपयोग करना पसंद करते हैं, तो निम्नलिखित कमांड के माध्यम से अपनी सेटिंग्स की जांच कर सकते हैं:
        ```bash
        yolo settings
        ```

### सेटिंग्स संशोधित करना

Ultralytics के सेटिंग्स को संशोधित करना आसान है। बदलावों को निम्न तरीकों से किया जा सकता है:

!!! Example "सेटिंग्स अपडेट करें"

    === "पायथन"
        पायथन पर्यावरण के भीतर, अपनी सेटिंग्स पर `अपडेट` विधि को बुलाकर अपनी सेटिंग्स को बदल सकते हैं:
        ```python
        from ultralytics import settings

        # एक सेटिंग अपडेट करें
        settings.update({'runs_dir': '/path/to/runs'})

        # एकाधिक सेटिंग अपडेट करें
        settings.update({'runs_dir': '/path/to/runs', 'tensorboard': False})

        # डिफ़ॉल्ट मान में सेटिंग रीसेट करें
        settings.reset()
        ```

    === "CLI"
        यदि आप कमांड लाइन इंटरफ़ेस पर ध्यान देते हैं, तो निम्नलिखित कमांड के माध्यम से अपनी सेटिंग्स को संशोधित कर सकते हैं:
        ```bash
        # एक सेटिंग अपडेट करें
        yolo settings runs_dir='/path/to/runs'

        # एकाधिक सेटिंग अपडेट करें
        yolo settings runs_dir='/path/to/runs' tensorboard=False

        # डिफ़ॉल्ट मान में सेटिंग्स को बराबरी में रीसेट करें
        yolo settings reset
        ```

### सेटिंग्स को समझना

निम्नलिखित टेबल सेटिंग्स का अवलोकन प्रदान करता है, जबकि प्रति सेटिंग्स के लिए उदाहरण मान, डेटा प्रकार और संक्षेप में विवरण दिया गया है।

| नाम                | उदाहरण मान            | डेटा प्रकार | विवरण                                                                                                                      |
|--------------------|-----------------------|-------------|----------------------------------------------------------------------------------------------------------------------------|
| `settings_version` | `'0.0.4'`             | `str`       | Ultralytics _settings_ संस्करण (Ultralytics [pip](https://pypi.org/project/ultralytics/) संस्करण से अलग होता है)           |
| `datasets_dir`     | `'/path/to/datasets'` | `str`       | डेटासेट को संग्रहीत करने वाली निर्देशिका                                                                                   | |
| `weights_dir`      | `'/path/to/weights'`  | `str`       | मॉडल वेट को संग्रहीत करने वाली निर्देशिका                                                                                  |
| `runs_dir`         | `'/path/to/runs'`     | `str`       | प्रयोग दौड़ को संग्रहीत करने वाली निर्देशिका                                                                               |
| `uuid`             | `'a1b2c3d4'`          | `str`       | मौजूदा सेटिंग्स के लिए अद्वितीय पहचानकर्ता                                                                                 |
| `sync`             | `True`                | `bool`      | Ultralytics और दुविधा को HUB में समकालीन रखें                                                                              |
| `api_key`          | `''`                  | `str`       | Ultralytics HUB [API Key](https://hub.ultralytics.com/settings?tab=api+keys)                                               |
| `clearml`          | `True`                | `bool`      | ClearML लॉगिंग का उपयोग करें                                                                                               |
| `comet`            | `True`                | `bool`      | यदि [Comet ML](https://bit.ly/yolov8-readme-comet) प्रयोग करें या नहीं experiment ट्रैकिंग और visualization                |
| `dvc`              | `True`                | `bool`      | शोध और संस्करण नियंत्रण के लिए [DVC for experiment tracking](https://dvc.org/doc/dvclive/ml-frameworks/yolo) का उपयोग करें |
| `hub`              | `True`                | `bool`      | [Ultralytics HUB](https://hub.ultralytics.com) एकीकरण का उपयोग करें                                                        |
| `mlflow`           | `True`                | `bool`      | एक्सपेरिमेंट ट्रैकिंग के लिए MLFlow का उपयोग करें                                                                          |
| `neptune`          | `True`                | `bool`      | एक्सपेरिमेंट ट्रैकिंग के लिए Neptune का उपयोग करें                                                                         |
| `raytune`          | `True`                | `bool`      | hyperparameter tuning के लिए Ray Tune का उपयोग करें                                                                        |
| `tensorboard`      | `True`                | `bool`      | विज़ुअलाइज़ेशन के लिए TensorBoard का उपयोग करें                                                                            |
| `wandb`            | `True`                | `bool`      | Weights & Biases logging का उपयोग करें                                                                                     |

जब आप अपने परियोजनाओं या अनुभागों के माध्यम से चलते होने के द्वारा यात्रा करते हैं, तो इन सेटिंग्स पर सुधार करने के लिए लौटें।
