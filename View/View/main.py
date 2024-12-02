import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
import csv


def parse_float(value, decimal_separator):
    """Парсить строку с учетом разделителя между целой и дробной частью."""
    if decimal_separator == ',':
        value = value.replace(',', '.')
    return float(value)


def read_data(file_path, decimal_separator='.'):
    """Чтение данных из файла и преобразование их в массивы x, y, T."""
    x_vals = []
    y_vals = []
    t_vals = []

    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter=' ')
        for row in reader:
            if len(row) != 3:
                continue  # Пропустить некорректные строки
            x = parse_float(row[0], decimal_separator)
            y = parse_float(row[1], decimal_separator)
            T = parse_float(row[2], decimal_separator)
            x_vals.append(x)
            y_vals.append(y)
            t_vals.append(T)

    return np.array(x_vals), np.array(y_vals), np.array(t_vals)


def plot_temperature_field(x, y, T):
    """Построение поля температуры на плоскости с интерполяцией."""

    # Создаем регулярную сетку для интерполяции
    grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]

    # Интерполяция данных температуры на регулярную сетку
    # method = linear, cubic
    grid_T = griddata((x, y), T, (grid_x, grid_y), method='nearest')

    # Ограничение интерполированных значений температуры в диапазоне от 0 до 500 градусов
    grid_T = np.clip(grid_T, 0, 300)

    # Построение графика
    plt.figure(figsize=(8, 6))
    plt.contourf(grid_x, grid_y, grid_T, levels=100, cmap='coolwarm')
    plt.colorbar(label='Temperature')
    # plt.scatter(x, y, c=T, cmap='coolwarm', edgecolors='black', s=50)  # точки измерений
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Temperature Field')
    plt.show()

def read_velocity_data(file_path, decimal_separator='.'):
    """Чтение данных из файла и преобразование их в массивы x, y, u_x, u_y."""
    x_vals = []
    y_vals = []
    u_x_vals = []
    u_y_vals = []

    with open(file_path, 'r') as file:
        reader = csv.reader(file, delimiter=' ')
        for row in reader:
            if len(row) != 4:
                continue  # Пропустить некорректные строки
            x = parse_float(row[0], decimal_separator)
            y = parse_float(row[1], decimal_separator)
            u_x = parse_float(row[2], decimal_separator)
            u_y = parse_float(row[3], decimal_separator)
            x_vals.append(x)
            y_vals.append(y)
            u_x_vals.append(u_x)
            u_y_vals.append(u_y)

    return np.array(x_vals), np.array(y_vals), np.array(u_x_vals), np.array(u_y_vals)


def plot_velocity_field(x, y, u_x, u_y):
    """Построение векторного поля скорости с управлением длиной стрелок."""
    plt.figure(figsize=(8, 6))

    scale_factor = 20  # Настройка масштаба длины стрелок
    u_x = u_x * scale_factor
    u_y = u_y * scale_factor

    # Построение векторов скорости с фиксированной шириной и наконечником
    plt.quiver(
        x, y, u_x, u_y,
        angles='xy',
        scale=1,  # Управление масштабом длины стрелок
        width=0.001,  # Толщина тела стрелки
        headwidth=7,  # Ширина наконечника стрелки
        headlength=7,  # Длина наконечника стрелки
        color='blue',
        alpha=0.8,
        pivot='tail'  # Стрелка начинается с заданной точки
    )

    # Настройка графика
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Velocity Field')
    plt.grid(True)
    plt.axis('equal')  # Одинаковый масштаб по осям
    plt.show()

def plot_heat_and_velocity_field(x, y, T, v_x, v_y, v_value_x, v_value_y):
    grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]

    # Интерполяция температуры
    grid_T = griddata((x, y), T, (grid_x, grid_y), method='cubic')
    grid_T = np.clip(grid_T, 0, 300)

    # Масштабирование длины стрелок
    scale_factor = 20
    v_value_x = v_value_x * scale_factor
    v_value_y = v_value_y * scale_factor

    # Построение графика
    plt.figure(figsize=(8, 6))

    # Построение температурного поля
    contour = plt.contourf(grid_x, grid_y, grid_T, levels=100, cmap='coolwarm')

    # Построение векторов скорости с фиксированным цветом
    plt.quiver(
        v_x, v_y, v_value_x, v_value_y,
        angles='xy',
        scale=1,  # Управление масштабом длины стрелок
        width=0.001,  # Толщина тела стрелки
        headwidth=5,  # Ширина наконечника стрелки
        headlength=5,  # Длина наконечника стрелки
        color='black',  # Фиксированный цвет стрелок
        pivot='tail',  # Стрелка начинается с заданной точки
    )

    # Добавление шкалы только для температурного поля
    plt.colorbar(contour, label='Temperature')  # Привязываем шкалу только к температуре
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Temperature and Velocity Field')
    plt.show()

decimal_separator = ','

file_path = 'data500.txt'
x, y, T = read_data(file_path, decimal_separator)

# plot_temperature_field(x, y, T)


velocity_file_path = 'velocity.txt'
v_point_x, v_point_y, u_x, u_y = read_velocity_data(velocity_file_path, decimal_separator)

# Построение поля скорости
# plot_velocity_field(x, y, u_x, u_y)
plot_heat_and_velocity_field(x, y, T, v_point_x, v_point_y, u_x, u_y)