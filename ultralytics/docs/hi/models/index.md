---
comments: true
description: Ultralytics द्वारा समर्थित YOLO परिवार की विविध रेंज, SAM, MobileSAM, FastSAM, YOLO-NAS, और RT-DETR मॉडल्स का पता लगाएं। CLI और Python उपयोग के लिए उदाहरणों के साथ प्रारंभ करें।
keywords: Ultralytics, दस्तावेज़ीकरण, YOLO, SAM, MobileSAM, FastSAM, YOLO-NAS, RT-DETR, मॉडल्स, आर्किटेक्चर्स, Python, CLI
---

# Ultralytics द्वारा समर्थित मॉडल

Ultralytics के मॉडल दस्तावेज़ीकरण में आपका स्वागत है! हम [ऑब्जेक्ट डिटेक्शन](../tasks/detect.md), [इंस्टेंस सेगमेंटेशन](../tasks/segment.md), [इमेज क्लासिफिकेशन](../tasks/classify.md), [पोज़ एस्टिमेशन](../tasks/pose.md), और [मल्टी-ऑब्जेक्ट ट्रैकिंग](../modes/track.md) जैसे विशिष्ट कामों के लिए डिज़ाइन किए गए मॉडलों की एक विस्तृत रेंज का समर्थन प्रदान करते हैं। यदि आप Ultralytics में अपने मॉडल आर्किटेक्चर को योगदान देने में रुचि रखते हैं, तो हमारा [Contributing Guide](../../help/contributing.md) देखें।

!!! Note "ध्यान दें"

    🚧 हमारी अलग-अलग भाषाओं में दस्तावेज़ीकरण वर्तमान में निर्माणाधीन है, और हम इसे सुधारने के लिए कठिन परिश्रम कर रहे हैं। धैर्य रखने के लिए धन्यवाद! 🙏

## प्रमुख मॉडल

यहां कुछ मुख्य मॉडल दिए गए हैं:

1. **[YOLOv3](yolov3.md)**: YOLO मॉडल परिवार का तीसरा संस्करण, जिसे जोसेफ रेडमोन द्वारा बनाया गया है, जो इसकी कुशल रियल-टाइम ऑब्जेक्ट डिटेक्शन क्षमताओं के लिए जाना जाता है।
2. **[YOLOv4](yolov4.md)**: YOLOv3 को अपडेट करने वाला एक डार्कनेट-नेटिव, जिसे 2020 में एलेक्सी बोचकोवस्की द्वारा जारी किया गया।
3. **[YOLOv5](yolov5.md)**: उल्ट्रालाइटिक्स द्वारा बेहतर YOLO आर्किटेक्चर का एक सुधारित संस्करण, जो पिछले संस्करणों की तुलना में बेहतर प्रदर्शन और गति की समझौता की पेशकश करता है।
4. **[YOLOv6](yolov6.md)**: 2022 में [Meituan](https://about.meituan.com/) द्वारा जारी किया गया, और कंपनी के कई स्वायत्त डिलीवरी रोबोट्स में उपयोग में।
5. **[YOLOv7](yolov7.md)**: 2022 में YOLOv4 के लेखकों द्वारा जारी किया गया अपडेटेड YOLO मॉडल।
6. **[YOLOv8](yolov8.md) नया 🚀**: YOLO परिवार का नवीनतम संस्करण, जिसमें इंस्टेंस सेगमेंटेशन, पोज/कीपॉइंट्स अनुमान, और क्लासिफिकेशन जैसी उन्नत क्षमताएं शामिल हैं।
7. **[Segment Anything Model (SAM)](sam.md)**: मेटा के Segment Anything Model (SAM)।
8. **[Mobile Segment Anything Model (MobileSAM)](mobile-sam.md)**: मोबाइल एप्लिकेशनों के लिए MobileSAM, क्युंग ही यूनिवर्सिटी द्वारा।
9. **[Fast Segment Anything Model (FastSAM)](fast-sam.md)**: चीनी विज्ञान अकादमी, ऑटोमेशन संस्थान के इमेज & वीडियो एनालिसिस ग्रुप द्वारा FastSAM।
10. **[YOLO-NAS](yolo-nas.md)**: YOLO न्यूरल आर्किटेक्चर सर्च (NAS) मॉडल्स।
11. **[Realtime Detection Transformers (RT-DETR)](rtdetr.md)**: बैदु के पडलपैडल Realtime Detection Transformer (RT-DETR) मॉडल।

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/MWq1UxqTClU?si=nHAW-lYDzrz68jR0"
    title="YouTube वीडियो प्लेयर" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>देखें:</strong> कुछ लाइनों के कोड में Ultralytics YOLO मॉडल्स को चलाएं।
</p>

## प्रारंभ करना: उपयोग उदाहरण

यह उदाहरण योलो प्रशिक्षण और अनुमान के सरल उदाहरण प्रदान करता है। इन और अन्य [modes](../modes/index.md) के पूर्ण दस्तावेज़ीकरण के लिए [Predict](../modes/predict.md), [Train](../modes/train.md), [Val](../modes/val.md) और [Export](../modes/export.md) दस्तावेज़ों के पन्नों को देखें।

नीचे दिया गया उदाहरण YOLOv8 [Detect](../tasks/detect.md) मॉडल्स के लिए है, जो ऑब्जेक्ट डिटेक्शन के लिए हैं। अतिरिक्त समर्थित कार्यों के लिए [Segment](../tasks/segment.md), [Classify](../tasks/classify.md) और [Pose](../tasks/pose.md) दस्तावेज़ों को देखें।

!!! Example "उदाहरण"

    === "Python"

        पायथन में मॉडल बनाने के लिए PyTorch प्रीट्रेन्ड '*.pt' मॉडल्स के साथ-साथ कॉन्फ़िगरेशन '*.yaml' फ़ाइलों को `YOLO()`, `SAM()`, `NAS()` और `RTDETR()` क्लासेज़ में पास किया जा सकता है:

        ```python
        from ultralytics import YOLO

        # COCO-प्रीट्रेन्ड YOLOv8n मॉडल लोड करें
        model = YOLO('yolov8n.pt')

        # मॉडल की जानकारी दिखाएँ (वैकल्पिक)
        model.info()

        # COCO8 उदाहरण डेटासेट पर 100 एपोक्स के लिए मॉडल प्रशिक्षित करें
        results = model.train(data='coco8.yaml', epochs=100, imgsz=640)

        # 'bus.jpg' इमेज पर YOLOv8n मॉडल के साथ अनुमान चलाएँ
        results = model('path/to/bus.jpg')
        ```

    === "CLI"

        CLI कमांड्स उपलब्ध हैं जो सीधे मॉडल्स को चलाने के लिए हैं:

        ```bash
        # COCO-प्रीट्रेन्ड YOLOv8n मॉडल को लोड करें और COCO8 उदाहरण डेटासेट पर 100 एपोक्स के लिए प्रशिक्षित करें
        yolo train model=yolov8n.pt data=coco8.yaml epochs=100 imgsz=640

        # COCO-प्रीट्रेन्ड YOLOv8n मॉडल को लोड करें और 'bus.jpg' इमेज पर अनुमान चलाएँ
        yolo predict model=yolov8n.pt source=path/to/bus.jpg
        ```

## नए मॉडल्स का योगदान

आप Ultralytics में अपने मॉडल का योगदान देने के इच्छुक हैं? बहुत बढ़िया! हम हमेशा अपने मॉडल पोर्टफोलियो का विस्तार करने के लिए खुले हैं।

1. **रिपॉजिटरी फोर्क करें**: [Ultralytics GitHub रिपॉजिटरी](https://github.com/ultralytics/ultralytics) को फोर्क करके शुरू करें।

2. **अपने फोर्क को क्लोन करें**: अपने फोर्क को अपनी लोकल मशीन पर क्लोन करें और काम करने के लिए एक नई ब्रांच बनाएं।

3. **अपना मॉडल लागू करें**: हमारे [Contributing Guide](../../help/contributing.md) में दिए गए कोडिंग स्टैंडर्ड्स और दिशानिर्देशों का अनुसरण करते हुए अपने मॉडल को जोड़ें।

4. **गहराई से परीक्षण करें**: अपने मॉडल का परीक्षण अलग से और पाइपलाइन के हिस्से के रूप में किया जा सकता है।

5. **पुल रिक्वेस्ट बनाएं**: एक बार जब आप अपने मॉडल से संतुष्ट हो जाएं, तो समीक्षा के लिए मुख्य रिपॉजिटरी को एक पुल रिक्वेस्ट बनाएं।

6. **कोड समीक्षा और मिलान**: समीक्षा के बाद, यदि आपका मॉडल हमारे मानदंडों को पूरा करता है, तो इसे मुख्य रिपॉजिटरी में मिला दिया जाएगा।

विस्तृत चरणों के लिए हमारा [Contributing Guide](../../help/contributing.md) देखें।
