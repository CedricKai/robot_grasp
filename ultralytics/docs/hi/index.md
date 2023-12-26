---
comments: true
description: Ultralytics YOLOv8 के पूर्ण गाइड को जानें, एक उच्च गति, उच्च योग्यता वाले वस्तु का पता लगाने और छवि विभाजन मॉडल। स्थापना, भविष्यवाणी, प्रशिक्षण ट्यूटोरियल और बहुत कुछ।
keywords: Ultralytics, YOLOv8, वस्तु पता लगाना, छवि विभाजन, मशीन लर्निंग, गहरी लर्निंग, कंप्यूटर विज़न, YOLOv8 स्थापना, YOLOv8 भविष्यवाणी, YOLOv8 प्रशिक्षण, YOLO इतिहास, YOLO लाइसेंसेस
---

<div align="center">
  <p>
    <a href="https://yolovision.ultralytics.com" target="_blank">
    <img width="1024" src="https://raw.githubusercontent.com/ultralytics/assets/main/yolov8/banner-yolov8.png" alt="Ultralytics YOLO banner"></a>
  </p>
  <a href="https://github.com/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-github.png" width="3%" alt="Ultralytics GitHub"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://www.linkedin.com/company/ultralytics/"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-linkedin.png" width="3%" alt="Ultralytics LinkedIn"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://twitter.com/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-twitter.png" width="3%" alt="Ultralytics Twitter"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://youtube.com/ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-youtube.png" width="3%" alt="Ultralytics YouTube"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://www.tiktok.com/@ultralytics"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-tiktok.png" width="3%" alt="Ultralytics TikTok"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://www.instagram.com/ultralytics/"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-instagram.png" width="3%" alt="Ultralytics Instagram"></a>
  <img src="https://github.com/ultralytics/assets/raw/main/social/logo-transparent.png" width="3%" alt="space">
  <a href="https://ultralytics.com/discord"><img src="https://github.com/ultralytics/assets/raw/main/social/logo-social-discord.png" width="3%" alt="Ultralytics Discord"></a>
  <br>
  <br>
  <a href="https://github.com/ultralytics/ultralytics/actions/workflows/ci.yaml"><img src="https://github.com/ultralytics/ultralytics/actions/workflows/ci.yaml/badge.svg" alt="Ultralytics CI"></a>
  <a href="https://codecov.io/github/ultralytics/ultralytics"><img src="https://codecov.io/github/ultralytics/ultralytics/branch/main/graph/badge.svg?token=HHW7IIVFVY" alt="Ultralytics Code Coverage"></a>
  <a href="https://zenodo.org/badge/latestdoi/264818686"><img src="https://zenodo.org/badge/264818686.svg" alt="YOLOv8 Citation"></a>
  <a href="https://hub.docker.com/r/ultralytics/ultralytics"><img src="https://img.shields.io/docker/pulls/ultralytics/ultralytics?logo=docker" alt="Docker Pulls"></a>
  <a href="https://ultralytics.com/discord"><img alt="Discord" src="https://img.shields.io/discord/1089800235347353640?logo=discord&logoColor=white&label=Discord&color=blue"></a>
  <br>
  <a href="https://console.paperspace.com/github/ultralytics/ultralytics"><img src="https://assets.paperspace.io/img/gradient-badge.svg" alt="Run on Gradient"></a>
  <a href="https://colab.research.google.com/github/ultralytics/ultralytics/blob/main/examples/tutorial.ipynb"><img src="https://colab.research.google.com/assets/colab-badge.svg" alt="Open In Colab"></a>
  <a href="https://www.kaggle.com/ultralytics/yolov8"><img src="https://kaggle.com/static/images/open-in-kaggle.svg" alt="Open In Kaggle"></a>
</div>


पेश करते हैं [युल्ट्रालिटिक्स](https://ultralytics.com) [YOLOv8](https://github.com/ultralytics/ultralytics), प्रसिद्ध वास्तविक समय वस्तु पता लगाने और छवि विभाजन मॉडल की नवीनतम संस्करण। YOLOv8 गहरी लर्निंग और कंप्यूटर विज़न में कटिंग-एज उन्नति पर आधारित है, इसलिए गति और योग्यता के मामले में इसका प्रदर्शन अद्वितीय है। इसका संक्षेपित डिज़ाइन इसे विभिन्न अनुप्रयोगों के लिए उपयुक्त बनाता है और विभिन्न हार्डवेयर प्लेटफ़ॉर्म्स पर आसानी से अनुकूल बनाता है, शुरू और धारण के लिए िजोग्य करता है।

YOLOv8 डॉक्स का अन्वेषण करें, यह एक व्यापक स्रोत है जो आपको इसके सुविधाओं और क्षमताओं को समझने और उपयोग करने में मदद करने के लिए विकसित किया गया है। चाहे आप एक अनुभवी मशीन लर्निंग प्रैक्टीशनर हो या क्षेत्र में नये हों, इस हब का उद्देश्य आपके परियोजनाओं में YOLOv8 की क्षमताओं को अधिकतम करना है।

!!! Note "नोट"

    🚧 हमारी बहुभाषी दस्तावेजीकरण वर्तमान में निर्माणाधीन है, और हम इसे सुधारने के लिए कठिनताओं पर काम कर रहे हैं। आपकी सहायता के लिए धन्यवाद! 🙏

## शुरुआत कहाँ से करें

- **Install** `pip` के साथ `ultralytics` स्थापित करें और कुछ मिनट में चलता हुआ पाएं &nbsp; [:material-clock-fast: शुरू हो जाओ](quickstart.md){ .md-button }
- **Predict** यूनिक images और videos को YOLOv8 के साथ &nbsp; [:octicons-image-16: छवियों पर भविष्यवाणी करें](modes/predict.md){ .md-button }
- **Train** अपने खुद के custom डेटासेट पर एक नया YOLOv8 मॉडल &nbsp; [:fontawesome-solid-brain: मॉडल प्रशिक्षित करें](modes/train.md){ .md-button }
- **अन्वेषण** करें YOLOv8 tasks जैसे कि विभाजित, वर्गीकृत, स्थिति और ट्रैक करें &nbsp; [:material-magnify-expand: टास्क्स अन्वेषण करें](tasks/index.md){ .md-button }

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/LNwODJXcvt4?si=7n1UvGRLSd9p5wKs"
    title="YouTube video player" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>देखें:</strong> अपने कस्टम डेटासेट पर YOLOv8 मॉडल को कैसे ट्रेन करें <a href="https://colab.research.google.com/github/ultralytics/ultralytics/blob/main/examples/tutorial.ipynb" target="_blank">Google Colab</a> में।
</p>

## YOLO: एक संक्षिप्त इतिहास

[YOLO](https://arxiv.org/abs/1506.02640) (You Only Look Once), एक लोकप्रिय वस्तु पता लगाने और छवि विभाजन मॉडल, यूनिवर्सिटी ऑफ वाशिंगटन में Joseph Redmon और Ali Farhadi द्वारा विकसित किया गया था। YOLO की उच्च गति और योग्यता के कारण, यह 2015 में तेजी से प्रसिद्ध हुआ।

- [YOLOv2](https://arxiv.org/abs/1612.08242), 2016 में जारी किया गया, मूल मॉडल में batch normalization, anchor boxes और dimension clusters शामिल करके मॉडल में सुधार किया।
- [YOLOv3](https://pjreddie.com/media/files/papers/YOLOv3.pdf), 2018 में लॉन्च किया गया, एक अधिक प्रभावी बैकबोन नेटवर्क, एंकर बॉक्सेस और स्थानिक पिरामिड पूलिंग के उपयोग से मॉडल की प्रदर्शन को और बढ़ाया।
- [YOLOv4](https://arxiv.org/abs/2004.10934) 2020 में जारी किया गया, Mosaic डेटा वृद्धि, एक नया anchor-free डिटेक्शन हेड और एक नया लॉस फ़ंक्शन के जैसे नवाचार द्वारा मॉडल को बेहतर बनाया गया।
- [YOLOv5](https://github.com/ultralytics/yolov5) मॉडल की प्रदर्शन को और बेहतर बनाने के साथ, हाइपरपैरामीटर ऑप्टिमाइज़ेशन, एकीकृत प्रयोग ट्रैकिंग और लोकप्रिय export formats में स्वचालित निर्यात जैसे नए सुविधाएं जोड़ी गईं।
- [YOLOv6](https://github.com/meituan/YOLOv6) 2022 में [मेटुआन](https://about.meituan.com/) द्वारा ओपन-सोस्ड किया गया था और कई कम्पनी के स्वतंत्र वितरण रोबोट में उपयोग में है।
- [YOLOv7](https://github.com/WongKinYiu/yolov7) ने COCO keypoints डेटासेट पर पोज अनुमान जैसे अतिरिक्त टास्क जोड़ दिया।
- [YOLOv8](https://github.com/ultralytics/ultralytics) Ultralytics द्वारा YOLO का नवीनतम संस्करण है। एक तलवार की काट, आपातता मॉडल के सफलता पर निर्मितकर्ताओं की मेहनत की चटानों पर निर्माण करके YOLOv8 ने पिछले संस्करणों की सफलता पर आधारित, नई सुविधाएं और सुधार अद्यतित प्रदर्शन, लचीलापन और प्रदार्थता के लिए प्रस्तुत किए हैं। YOLOv8 विजन AI tasks, जैसे [पता लगाना](tasks/detect.md), [विभाजन](tasks/segment.md), [पोज अनुमान](tasks/pose.md), [ट्रैकिंग](modes/track.md), और [वर्गीकरण](tasks/classify.md) का पूरा समर्थन करता है। यह विविध अनुप्रयोग और क्षेत्रों में योलोवी8 की क्षमताओं का उपयोग करने की अनुमति देता है।

YOLO लाइसेंसेस: Ultralytics YOLO का प्रयोग कैसे होता है?

Ultralytics विभिन्न उपयोग मामलों को समर्थित करने के लिए दो लाइसेंसिंग विकल्प प्रदान करता है:

- **AGPL-3.0 लाइसेंस**: यह [OSI स्वीकृत](https://opensource.org/licenses/) ओपन-सोर्स लाइसेंस छात्रों और उत्साहीयों के लिए उपयुक्त है, गहन सहयोग और ज्ञान साझा करने के लिए प्रोत्साहित करता है। अधिक जानकारी के लिए [LICENSE](https://github.com/ultralytics/ultralytics/blob/main/LICENSE) फ़ाइल देखें।
- **व्यवसायिक लाइसेंस**: व्यावसायिक उपयोग के लिए डिज़ाइन किया गया, यह लाइसेंस Ultralytics सॉफ़्टवेयर और AI मॉडल को वाणिज्यिक माल और सेवाओं में सरलतापूर्वक सम्मिलित करने की अनुमति देता है, AGPL-3.0 की ओपन-सोर्स आवश्यकताओं को छोड़ता है। यदि आपके परिदृश्य में हमारे समाधानों को एक वाणिज्यिक प्रस्ताव में एम्बेड करना शामिल है, [Ultralytics Licensing](https://ultralytics.com/license) के माध्यम से संपर्क करें।

हमारी लाइसेंसिंग रणनीति इस सुनिश्चित करने के लिए है कि हमारे ओपन-सोर्स परियोजनाओं में किए गए कोई भी सुधार समुदाय को लौटाए जाएं। हम ओपन सोर्स के सिद्धांतों को अपने दिल के पास रखते हैं ❤️, और हमारा मिशन यह सुनिश्चित करना है कि हमारे योगदानों का उपयोग और विस्तार किए जाने के तरीकों में क्रियान्वयन किए जाएं जो सभी के लिए लाभदायक हों।
