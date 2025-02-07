import vtk
import serial
import numpy as np
import threading
import time
from vtk.util import numpy_support

class PaddleVisualizer:
    def __init__(self, port="/dev/cu.usbserial-0001", baudrate=115200):
        # Инициализация Serial порта
        self.serial = serial.Serial(port, baudrate)
        self.kayak_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.paddle_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Создание VTK рендерера
        self.renderer = vtk.vtkRenderer()
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.AddRenderer(self.renderer)
        self.render_window.SetSize(800, 600)
        
        # Создание интерактора
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.render_window)
        
        # Создание объектов
        self.setup_kayak()
        self.setup_paddle()
        
        # Настройка камеры
        self.setup_camera()
        
        # Запуск потока чтения данных
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_data)
        self.read_thread.daemon = True
        
    def setup_kayak(self):
        # Создаем точки для формы каяка
        points = vtk.vtkPoints()
        
        # Нижние точки
        points.InsertNextPoint(1.5, 0.0, -0.2)     # 0: нос
        points.InsertNextPoint(-1.5, -0.2, -0.2)   # 1: корма левая
        points.InsertNextPoint(-1.5, 0.2, -0.2)    # 2: корма правая
        
        # Верхние точки
        points.InsertNextPoint(1.5, 0.0, 0.2)      # 3: нос верх
        points.InsertNextPoint(-1.5, -0.3, 0.2)    # 4: корма верх левая
        points.InsertNextPoint(-1.5, 0.3, 0.2)     # 5: корма верх правая
        
        # Точки для бортов
        points.InsertNextPoint(0.0, -0.3, -0.1)    # 6: середина низ левая
        points.InsertNextPoint(0.0, 0.3, -0.1)     # 7: середина низ правая
        points.InsertNextPoint(0.0, -0.4, 0.2)     # 8: середина верх левая
        points.InsertNextPoint(0.0, 0.4, 0.2)      # 9: середина верх правая

        # Создаем полигоны
        polygons = vtk.vtkCellArray()
        
        # Дно
        self.add_polygon(polygons, [0, 1, 2])  # треугольное дно носа
        self.add_polygon(polygons, [0, 6, 1])  # левая часть дна
        self.add_polygon(polygons, [0, 2, 7])  # правая часть дна
        self.add_polygon(polygons, [6, 7, 1, 2])  # дно кормы
        
        # Борта
        self.add_polygon(polygons, [0, 3, 8, 6])  # левый борт нос
        self.add_polygon(polygons, [0, 7, 9, 3])  # правый борт нос
        self.add_polygon(polygons, [6, 8, 4, 1])  # левый борт корма
        self.add_polygon(polygons, [7, 9, 5, 2])  # правый борт корма
        
        # Палуба
        self.add_polygon(polygons, [3, 8, 9])  # палуба нос
        self.add_polygon(polygons, [8, 4, 5, 9])  # палуба корма
        
        # Корма
        self.add_polygon(polygons, [1, 4, 5, 2])  # транец

        # Создаем модель
        kayak_pd = vtk.vtkPolyData()
        kayak_pd.SetPoints(points)
        kayak_pd.SetPolys(polygons)

        # Добавляем нормали для правильного освещения
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(kayak_pd)
        normals.ConsistencyOn()
        normals.SplittingOff()
        
        # Маппер и актор
        kayak_mapper = vtk.vtkPolyDataMapper()
        kayak_mapper.SetInputConnection(normals.GetOutputPort())
        
        self.kayak_actor = vtk.vtkActor()
        self.kayak_actor.SetMapper(kayak_mapper)
        self.kayak_actor.GetProperty().SetColor(0.2, 0.2, 0.8)  # синий цвет
        
        self.renderer.AddActor(self.kayak_actor)

    def add_polygon(self, cell_array, point_ids):
        polygon = vtk.vtkPolygon()
        polygon.GetPointIds().SetNumberOfIds(len(point_ids))
        for i, pid in enumerate(point_ids):
            polygon.GetPointIds().SetId(i, pid)
        cell_array.InsertNextCell(polygon)

    def setup_paddle(self):
        # Создание весла
        # Шафт весла
        shaft = vtk.vtkCylinderSource()
        shaft.SetHeight(2.0)
        shaft.SetRadius(0.03)
        shaft.SetResolution(20)
        
        # Лопасти весла
        blade_left = vtk.vtkCubeSource()
        blade_left.SetXLength(0.02)
        blade_left.SetYLength(0.2)
        blade_left.SetZLength(0.4)
        
        blade_right = vtk.vtkCubeSource()
        blade_right.SetXLength(0.02)
        blade_right.SetYLength(0.2)
        blade_right.SetZLength(0.4)
        
        # Трансформации для лопастей
        transform_left = vtk.vtkTransform()
        transform_left.Translate(0, -1.0, 0)  # сдвиг влево
        
        transform_right = vtk.vtkTransform()
        transform_right.Translate(0, 1.0, 0)  # сдвиг вправо
        
        # Применение трансформаций
        transform_filter_left = vtk.vtkTransformPolyDataFilter()
        transform_filter_left.SetInputConnection(blade_left.GetOutputPort())
        transform_filter_left.SetTransform(transform_left)
        
        transform_filter_right = vtk.vtkTransformPolyDataFilter()
        transform_filter_right.SetInputConnection(blade_right.GetOutputPort())
        transform_filter_right.SetTransform(transform_right)
        
        # Объединение частей весла
        append = vtk.vtkAppendPolyData()
        append.AddInputConnection(shaft.GetOutputPort())
        append.AddInputConnection(transform_filter_left.GetOutputPort())
        append.AddInputConnection(transform_filter_right.GetOutputPort())
        
        # Маппер и актор для весла
        paddle_mapper = vtk.vtkPolyDataMapper()
        paddle_mapper.SetInputConnection(append.GetOutputPort())
        
        self.paddle_actor = vtk.vtkActor()
        self.paddle_actor.SetMapper(paddle_mapper)
        self.paddle_actor.GetProperty().SetColor(0.8, 0.2, 0.2)  # красный цвет
        
        self.renderer.AddActor(self.paddle_actor)
        
    def setup_camera(self):
        camera = self.renderer.GetActiveCamera()
        camera.SetPosition(0, -10, 5)
        camera.SetFocalPoint(0, 0, 0)
        camera.SetViewUp(0, 0, 1)
        self.renderer.ResetCamera()
        
    def quaternion_to_transform(self, q):
        # Преобразование кватерниона в матрицу поворота
        w, x, y, z = q
        
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
        
    def read_serial_data(self):
        while self.running:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('Kayak'):
                    _, w, x, y, z = line.split(',')
                    self.kayak_quaternion = np.array([float(w), float(x), float(y), float(z)])
                elif line.startswith('Paddle'):
                    _, w, x, y, z = line.split(',')
                    self.paddle_quaternion = np.array([float(w), float(x), float(y), float(z)])
            except:
                continue
                
    def update_visualization(self, obj, event):
        # Обновление положения каяка
        kayak_transform = self.quaternion_to_transform(self.kayak_quaternion)
        self.kayak_actor.SetUserTransform(kayak_transform)
        
        # Обновление положения весла
        paddle_transform = self.quaternion_to_transform(self.paddle_quaternion)
        self.paddle_actor.SetUserTransform(paddle_transform)
        
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

if __name__ == "__main__":
    visualizer = PaddleVisualizer()
    try:
        visualizer.run()
    finally:
        visualizer.cleanup() 