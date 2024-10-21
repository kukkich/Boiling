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


# Основная программа
file_path = 'data500.txt'  # Укажите путь к вашему файлу
decimal_separator = ','  # Укажите разделитель для чисел, например ',' или '.'

# Чтение данных из файла
x, y, T = read_data(file_path, decimal_separator)

# Построение поля температуры
plot_temperature_field(x, y, T)