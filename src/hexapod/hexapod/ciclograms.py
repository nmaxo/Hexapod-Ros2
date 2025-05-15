from geometry_msgs.msg import Point

# Правильная инициализация точек
air_phase = (
    Point(x=500.0, y=500.0, z=600.0),  # Все значения должны быть float!
    Point(x=550.0, y=550.0, z=550.0),
    Point(x=500.0, y=500.0, z=600.0)
    )

stance_phase = (
    Point(x=550.0, y=500.0 ,z=600.0),
    Point(x=500.0, y=500.0 ,z=600.0),
    Point(x=450.0, y=500.0, z=600.0)
    )