import vtk
import serial
import numpy as np
import threading
import time
from vtk.util import numpy_support

class OrientationVisualizer:
    def __init__(self, port="/dev/cu.usbserial-0001", baudrate=115200):
        # Инициализация Serial порта
        self.serial = serial.Serial(port, baudrate)
        self.dmp_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.fusion_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.mag_vector = np.array([1.0, 0.0, 0.0])
        self.accel_vector = np.array([0.0, 0.0, 1.0])  # добавляем вектор акселерометра
        
        # Создание VTK рендерера
        self.renderer = vtk.vtkRenderer()
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.render_window.SetSize(800, 600)
        
        # Создание интерактора
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.render_window)
        
        # Создание объектов
        self.setup_dmp_stick()
        self.setup_fusion_stick()
        self.setup_mag_arrow()
        self.setup_accel_arrow()  # добавляем стрелку акселерометра
        
        # Настройка камеры
        self.setup_camera()
        
        # Запуск потока чтения данных
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        
        self.add_coordinate_axes()  # добавляем оси координат
        
    def create_stick(self, color):
        # Создание палки
        stick = vtk.vtkCylinderSource()
        stick.SetHeight(2.0)  # длина
        stick.SetRadius(0.05)  # радиус
        stick.SetResolution(20)  # количество граней
        
        # Поворачиваем цилиндр на 90° вокруг Z, чтобы он был вдоль X,
        # затем на 180° вокруг Z, чтобы учесть, что X датчика направлен на корму
        transform = vtk.vtkTransform()
        transform.RotateZ(-90)  # сначала вдоль X
        transform.RotateZ(180)  # разворот на 180°
        
        transformFilter = vtk.vtkTransformPolyDataFilter()
        transformFilter.SetInputConnection(stick.GetOutputPort())
        transformFilter.SetTransform(transform)
        
        # Маппер
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transformFilter.GetOutputPort())
        
        # Актор
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color)  # установка цвета
        
        return actor
        
    def setup_dmp_stick(self):
        self.dmp_actor = self.create_stick((0.2, 0.2, 0.8))  # синий
        self.renderer.AddActor(self.dmp_actor)
        
    def setup_fusion_stick(self):
        self.fusion_actor = self.create_stick((0.2, 0.8, 0.2))  # зеленый
        # Сдвигаем вторую палку немного вверх для лучшей видимости
#        self.fusion_actor.SetPosition(0, 0, 0.2)
        self.renderer.AddActor(self.fusion_actor)
        
    def setup_camera(self):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0, -5, 2)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()
        
    def quaternion_to_transform(self, q):
        # Преобразование кватерниона в матрицу поворота
        w, x, y, z = q
        
        # Инвертируем X и Y компоненты кватерниона, чтобы учесть разворот на 180°
        x = -x
        y = -y
        
        transform = vtk.vtkTransform()
        matrix = vtk.vtkMatrix4x4()
        
        # Заполнение матрицы
        matrix.SetElement(0, 0, 1 - 2*y*y - 2*z*z)
        matrix.SetElement(0, 1, 2*x*y - 2*w*z)
        matrix.SetElement(0, 2, 2*x*z + 2*w*y)
        matrix.SetElement(0, 3, 0)
        
        matrix.SetElement(1, 0, 2*x*y + 2*w*z)
        matrix.SetElement(1, 1, 1 - 2*x*x - 2*z*z)
        matrix.SetElement(1, 2, 2*y*z - 2*w*x)
        matrix.SetElement(1, 3, 0)
        
        matrix.SetElement(2, 0, 2*x*z - 2*w*y)
        matrix.SetElement(2, 1, 2*y*z + 2*w*x)
        matrix.SetElement(2, 2, 1 - 2*x*x - 2*y*y)
        matrix.SetElement(2, 3, 0)
        
        matrix.SetElement(3, 0, 0)
        matrix.SetElement(3, 1, 0)
        matrix.SetElement(3, 2, 0)
        matrix.SetElement(3, 3, 1)
        
        transform.SetMatrix(matrix)
        return transform
        
    def setup_mag_arrow(self):
        # Создаем стрелку
        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(20)
        arrow.SetShaftResolution(20)
        
        # Маппер и актор для стрелки
        mag_mapper = vtk.vtkPolyDataMapper()
        mag_mapper.SetInputConnection(arrow.GetOutputPort())
        
        self.mag_actor = vtk.vtkActor()
        self.mag_actor.SetMapper(mag_mapper)
        self.mag_actor.GetProperty().SetColor(1.0, 1.0, 0.0)  # желтый цвет
        
        self.renderer.AddActor(self.mag_actor)

    def setup_accel_arrow(self):
        # Создаем стрелку
        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(20)
        arrow.SetShaftResolution(20)
        
        # Маппер и актор для стрелки
        accel_mapper = vtk.vtkPolyDataMapper()
        accel_mapper.SetInputConnection(arrow.GetOutputPort())
        
        self.accel_actor = vtk.vtkActor()
        self.accel_actor.SetMapper(accel_mapper)
        self.accel_actor.GetProperty().SetColor(0.8, 0.0, 0.8)  # фиолетовый цвет
        
        self.renderer.AddActor(self.accel_actor)

    def update_mag_arrow(self):
        # Создаем трансформацию для стрелки
        transform = vtk.vtkTransform()
        
        # Вычисляем длину вектора
        mag_length = np.linalg.norm(self.mag_vector)
        
        # Нормализуем вектор для направления
        mag_norm = self.mag_vector / mag_length
        
        # Масштабируем стрелку по длине вектора (вместо фиксированного 0.5)
        transform.Scale(mag_length, mag_length, mag_length)
        
        # Вычисляем углы поворота
        z_angle = np.degrees(np.arctan2(mag_norm[1], mag_norm[0]))
        y_angle = np.degrees(np.arctan2(-mag_norm[2], 
                            np.sqrt(mag_norm[0]**2 + mag_norm[1]**2)))
        
        # Применяем повороты
        transform.RotateZ(z_angle)
        transform.RotateY(y_angle)
        
        self.mag_actor.SetUserTransform(transform)

    def update_accel_arrow(self):
        # Создаем трансформацию для стрелки
        transform = vtk.vtkTransform()
        
        # Вычисляем длину вектора
        accel_length = np.linalg.norm(self.accel_vector)
        
        # Нормализуем вектор для направления
        accel_norm = self.accel_vector / accel_length
        
        # Масштабируем стрелку по длине вектора (вместо фиксированного 0.5)
        transform.Scale(accel_length, accel_length, accel_length)
        
        # Вычисляем углы поворота
        z_angle = np.degrees(np.arctan2(accel_norm[1], accel_norm[0]))
        y_angle = np.degrees(np.arctan2(-accel_norm[2], 
                            np.sqrt(accel_norm[0]**2 + accel_norm[1]**2)))
        
        # Применяем повороты
        transform.RotateZ(z_angle)
        transform.RotateY(y_angle)
        
        self.accel_actor.SetUserTransform(transform)

    def read_serial_data(self):
        while self.running:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('DMP'):
                    _, w, x, y, z = line.split(',')
                    self.dmp_quaternion = np.array([float(w), float(x), float(y), float(z)])
                elif line.startswith('Fusion'):
                    _, w, x, y, z = line.split(',')
                    self.fusion_quaternion = np.array([float(w), float(x), float(y), float(z)])
                elif line.startswith('Mag'):
                    _, x, y, z = line.split(',')
                    self.mag_vector = np.array([float(x), float(y), float(z)])
                elif line.startswith('Accel'):
                    _, x, y, z = line.split(',')
                    self.accel_vector = np.array([float(x), float(y), float(z)])
            except Exception as e:
                print(f"Error reading serial: {e}")
                continue
                
    def update_visualization(self, obj, event):
        # Обновление положения DMP палки
        dmp_transform = self.quaternion_to_transform(self.dmp_quaternion)
        self.dmp_actor.SetUserTransform(dmp_transform)
        
        # Обновление положения Fusion палки
        fusion_transform = self.quaternion_to_transform(self.fusion_quaternion)
        # Сохраняем смещение по Z при обновлении трансформации
        pos = self.fusion_actor.GetPosition()
#        fusion_transform.Translate(0, 0, pos[2])
        self.fusion_actor.SetUserTransform(fusion_transform)
        
        # Обновление стрелок
        self.update_mag_arrow()
        self.update_accel_arrow()
        
        self.render_window.Render()
        
    def run(self):
        # Запуск потока чтения данных
        self.read_thread.start()
        
        # Добавление таймера для обновления визуализации
        self.interactor.Initialize()
        self.interactor.AddObserver('TimerEvent', self.update_visualization)
        self.interactor.CreateRepeatingTimer(int(1000/30))  # 30 Hz
        
        # Запуск визуализации
        self.render_window.Render()
        self.interactor.Start()
        
    def cleanup(self):
        self.running = False
        self.read_thread.join()
        self.serial.close()

    def add_coordinate_axes(self):
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.0, 1.0, 1.0)  # длина осей
        axes.SetShaftType(0)  # тип стрелок
        axes.SetAxisLabels(1)  # показывать подписи осей
        self.renderer.AddActor(axes)

if __name__ == "__main__":
    visualizer = OrientationVisualizer()
    try:
        visualizer.run()
    finally:
        visualizer.cleanup() 