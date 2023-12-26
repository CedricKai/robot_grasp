---
comments: true
description: Узнайте, как использовать Ultralytics YOLO для отслеживания объектов в видеопотоках. Руководства по использованию различных трекеров и настройке конфигурации трекера.
keywords: Ultralytics, YOLO, отслеживание объектов, видеопотоки, BoT-SORT, ByteTrack, руководство на Python, руководство CLI
---

# Множественное отслеживание объектов с помощью Ultralytics YOLO

<img width="1024" src="https://user-images.githubusercontent.com/26833433/243418637-1d6250fd-1515-4c10-a844-a32818ae6d46.png" alt="Примеры множественного отслеживания объектов">

Отслеживание объектов в сфере видеоаналитики является ключевой задачей, которая определяет не только местоположение и класс объектов в кадре, но также поддерживает уникальный ID для каждого обнаруженного объекта по мере развития видео. Приложения безграничны — от наблюдения и безопасности до аналитики реального времени в спорте.

## Почему стоит выбрать Ultralytics YOLO для отслеживания объектов?

Вывод с трекеров Ultralytics согласуется со стандартным обнаружением объектов, но имеет добавленные ID объектов. Это упрощает отслеживание объектов в видеопотоках и выполнение последующей аналитики. Вот почему вы должны рассмотреть использование Ultralytics YOLO для ваших потребностей в отслеживании объектов:

- **Эффективность:** Обработка видеопотоков в режиме реального времени без потери точности.
- **Гибкость:** Поддержка множества алгоритмов отслеживания и конфигураций.
- **Простота использования:** Простой Python API и CLI-опции для быстрой интеграции и развертывания.
- **Настраиваемость:** Легкость использования с пользовательскими обученными моделями YOLO, позволяющая интеграцию в специфические для домена приложения.

<p align="center">
  <br>
  <iframe width="720" height="405" src="https://www.youtube.com/embed/hHyHmOtmEgs?si=VNZtXmm45Nb9s-N-"
    title="YouTube видео плеер" frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <br>
  <strong>Смотрите:</strong> Обнаружение объектов и отслеживание с Ultralytics YOLOv8.
</p>

## Прикладные применения

|                                                                Транспорт                                                                |                                                          Ритейл                                                          |                                                      Аквакультура                                                      |
|:---------------------------------------------------------------------------------------------------------------------------------------:|:------------------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------------:|
| ![Отслеживание транспортных средств](https://github.com/RizwanMunawar/ultralytics/assets/62513924/ee6e6038-383b-4f21-ac29-b2a1c7d386ab) | ![Отслеживание людей](https://github.com/RizwanMunawar/ultralytics/assets/62513924/93bb4ee2-77a0-4e4e-8eb6-eb8f527f0527) | ![Отслеживание рыб](https://github.com/RizwanMunawar/ultralytics/assets/62513924/a5146d0f-bfa8-4e0a-b7df-3c1446cd8142) |
|                                                    Отслеживание транспортных средств                                                    |                                                    Отслеживание людей                                                    |                                                    Отслеживание рыб                                                    |

## Ключевые особенности

Ultralytics YOLO расширяет свои возможности обнаружения объектов для обеспечения надежного и универсального отслеживания объектов:

- **Отслеживание в реальном времени:** Безпрерывное отслеживание объектов в видео с высокой частотой кадров.
- **Поддержка множества трекеров:** Выбор из разнообразия установленных алгоритмов отслеживания.
- **Настраиваемые конфигурации трекеров:** Настройка алгоритма отслеживания для конкретных требований путем регулировки различных параметров.

## Доступные трекеры

Ultralytics YOLO поддерживает следующие алгоритмы отслеживания. Их можно включить, передав соответствующий YAML файл конфигурации, например `tracker=tracker_type.yaml`:

* [BoT-SORT](https://github.com/NirAharon/BoT-SORT) - Используйте `botsort.yaml`, чтобы активировать этот трекер.
* [ByteTrack](https://github.com/ifzhang/ByteTrack) - Используйте `bytetrack.yaml`, чтобы активировать этот трекер.

Трекер по умолчанию - BoT-SORT.

## Отслеживание

Для запуска трекера на видеопотоках используйте обученные модели Detect, Segment или Pose, такие как YOLOv8n, YOLOv8n-seg и YOLOv8n-pose.

!!! Example "Пример"

    === "Python"

        ```python
        from ultralytics import YOLO

        # Загрузите официальную или пользовательскую модель
        model = YOLO('yolov8n.pt')  # Загрузить официальную модель Detect
        model = YOLO('yolov8n-seg.pt')  # Загрузить официальную модель Segment
        model = YOLO('yolov8n-pose.pt')  # Загрузить официальную модель Pose
        model = YOLO('path/to/best.pt')  # Загрузить пользовательскую обученную модель

        # Выполнить отслеживание с помощью модели
        results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True)  # Отслеживание с трекером по умолчанию
        results = model.track(source="https://youtu.be/LNwODJXcvt4", show=True, tracker="bytetrack.yaml")  # Отслеживание с трекером ByteTrack
        ```

    === "CLI"

        ```bash
        # Выполнить отслеживание с различными моделями используя командный интерфейс
        yolo track model=yolov8n.pt source="https://youtu.be/LNwODJXcvt4"  # Официальная модель Detect
        yolo track model=yolov8n-seg.pt source="https://youtu.be/LNwODJXcvt4"  # Официальная модель Segment
        yolo track model=yolov8n-pose.pt source="https://youtu.be/LNwODJXcvt4"  # Официальная модель Pose
        yolo track model=path/to/best.pt source="https://youtu.be/LNwODJXcvt4"  # Пользовательская обученная модель

        # Отслеживание с использованием трекера ByteTrack
        yolo track model=path/to/best.pt tracker="bytetrack.yaml"
        ```

Как видно из вышеуказанного использования, отслеживание доступно для всех моделей Detect, Segment и Pose, работающих с видео или потоковыми источниками.

## Конфигурация

### Аргументы для отслеживания

Конфигурация отслеживания имеет общие свойства с режимом Predict, такие как `conf`, `iou` и `show`. Для дальнейшей настройки обратитесь к странице модели [Predict](https://docs.ultralytics.com/modes/predict/).

!!! Example "Пример"

    === "Python"

        ```python
        from ultralytics import YOLO

        # Настройте параметры отслеживания и запустите трекер
        model = YOLO('yolov8n.pt')
        results = model.track(source="https://youtu.be/LNwODJXcvt4", conf=0.3, iou=0.5, show=True)
        ```

    === "CLI"

        ```bash
        # Настройте параметры отслеживания и запустите трекер, используя командный интерфейс
        yolo track model=yolov8n.pt source="https://youtu.be/LNwODJXcvt4" conf=0.3, iou=0.5 show
        ```

### Выбор трекера

Ultralytics также позволяет использовать измененный файл конфигурации трекера. Для этого просто сделайте копию файла конфигурации трекера (например, `custom_tracker.yaml`) из [ultralytics/cfg/trackers](https://github.com/ultralytics/ultralytics/tree/main/ultralytics/cfg/trackers) и измените любые настройки (кроме `tracker_type`) в соответствии с вашими потребностями.

!!! Example "Пример"

    === "Python"

        ```python
        from ultralytics import YOLO

        # Загрузите модель и запустите трекер с пользовательским файлом конфигурации
        model = YOLO('yolov8n.pt')
        results = model.track(source="https://youtu.be/LNwODJXcvt4", tracker='custom_tracker.yaml')
        ```

    === "CLI"

        ```bash
        # Загрузите модель и запустите трекер с пользовательским файлом конфигурации, используя командный интерфейс
        yolo track model=yolov8n.pt source="https://youtu.be/LNwODJXcvt4" tracker='custom_tracker.yaml'
        ```

Для полного списка аргументов отслеживания обратитесь к странице [ultralytics/cfg/trackers](https://github.com/ultralytics/ultralytics/tree/main/ultralytics/cfg/trackers).

## Примеры на Python

### Цикл сохранения следов

Вот пример скрипта Python, использующий OpenCV (`cv2`) и YOLOv8 для выполнения отслеживания объектов на кадрах видео. В этом сценарии предполагается, что вы уже установили необходимые пакеты (`opencv-python` и `ultralytics`). Аргумент `persist=True` указывает трекеру, что текущее изображение или кадр является следующим в последовательности и ожидает, что следы с предыдущего изображения будут присутствовать в текущем изображении.

!!! Example "Цикл с потоковым отслеживанием for-loop"

    ```python
    import cv2
    from ultralytics import YOLO

    # Загрузите модель YOLOv8
    model = YOLO('yolov8n.pt')

    # Откройте видеофайл
    video_path = "path/to/video.mp4"
    cap = cv2.VideoCapture(video_path)

    # Цикл по кадрам видео
    while cap.isOpened():
        # Чтение кадра из видео
        success, frame = cap.read()

        if success:
            # Выполните отслеживание YOLOv8 для кадра, сохраняя следы между кадрами
            results = model.track(frame, persist=True)

            # Визуализируйте результаты на кадре
            annotated_frame = results[0].plot()

            # Покажите аннотированный кадр
            cv2.imshow("Отслеживание YOLOv8", annotated_frame)

            # Прервать цикл, если нажата клавиша 'q'
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        else:
            # Прервать цикл, если достигнут конец видео
            break

    # Освободите объект захвата видео и закройте окно отображения
    cap.release()
    cv2.destroyAllWindows()
    ```

Обратите внимание на изменение с `model(frame)` на `model.track(frame)`, которое позволяет включить отслеживание объектов вместо простого обнаружения. Этот измененный скрипт будет выполнять трекер на каждом кадре видео, визуализировать результаты и отображать их в окне. Цикл можно завершить нажатием 'q'.

## Содействие в новых трекерах

Вы являетесь профессионалом в множественном отслеживании объектов и успешно реализовали или адаптировали алгоритм отслеживания с Ultralytics YOLO? Мы приглашаем вас внести свой вклад в наш раздел Trackers на [ultralytics/cfg/trackers](https://github.com/ultralytics/ultralytics/tree/main/ultralytics/cfg/trackers)! Ваши реальные приложения и решения могут быть бесценными для пользователей, работающих над задачами отслеживания.

Внося свой вклад в этот раздел, вы помогаете расширить спектр доступных решений для отслеживания в рамках фреймворка Ultralytics YOLO, добавляя еще один уровень функциональности и полезности для сообщества.

Чтобы начать свой вклад, пожалуйста, ознакомьтесь с нашим [Руководством для участников](https://docs.ultralytics.com/help/contributing) для получения полной инструкции по отправке Pull Request (PR) 🛠️. Мы в предвкушении увидеть, что вы принесете на стол!

Вместе давайте улучшим возможности отслеживания экосистемы Ultralytics YOLO 🙏!
