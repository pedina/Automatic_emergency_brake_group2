import math

def calculate_universal_ttc(distance, v_ego, a_ego, v_target, a_target, accel_threshold=1e-5):
    """
    Kiszámítja az univerzális ütközésig hátralévő időt (TTC) a relatív mozgás alapján.
    
    Paraméterek:
    - distance (float): A kezdeti távolság az ego és a target jármű között (méter, > 0).
    - v_ego, v_target (float): Az ego és a target sebessége (m/s).
    - a_ego, a_target (float): Az ego és a target gyorsulása (m/s²).
    - accel_threshold (float): Küszöbérték, amely alatt a relatív gyorsulást elhanyagolhatónak (konstans sebességnek) tekintjük.
    
    Visszatérési érték:
    - float: A TTC értéke másodpercben, vagy float('inf') (végtelen), ha nem várható ütközés.
    
    Képlet: A 0.5 * a_rel * t² + v_rel * t - distance = 0 másodfokú egyenletet oldja meg, feltéve hogy a_rel != 0.
    """
    if distance <= 0:
        return 0.0  # Már megtörtént az ütközés
    
    rel_velocity = v_ego - v_target
    rel_acceleration = a_ego - a_target
    
    # 1. ESET: Nincs (vagy elhanyagolható) relatív gyorsulás
    if abs(rel_acceleration) < accel_threshold:
        if rel_velocity <= 0:
            return float('inf')  # Távolodunk vagy tartjuk a távolságot
        return distance / rel_velocity
    
    # 2. ESET: Van relatív gyorsulás (másodfokú egyenlet)
    # Képlet 0-ra rendezve: 0.5 * rel_acceleration * t^2 + rel_velocity * t - distance = 0
    
    # Diszkrimináns: b^2 - 4ac -> rel_velocity^2 - 4 * (0.5 * rel_acceleration) * (-distance)
    discriminant = rel_velocity**2 + 2 * rel_acceleration * distance
    
    if discriminant < 0:
        return float('inf')  # Ha a diszkrimináns negatív, nincs ütközés
    
    # Két lehetséges időpont kiszámítása
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-rel_velocity + sqrt_disc) / rel_acceleration
    t2 = (-rel_velocity - sqrt_disc) / rel_acceleration
    
    # Csak a jövőbeli (pozitív) időpontok számítanak
    valid_times = [t for t in (t1, t2) if t > 0]
    return min(valid_times) if valid_times else float('inf')

if __name__ == "__main__":
    d = 50.0  
    ve = 20.0 
    
    print(f"1. (Ego 20, Target áll):    {calculate_universal_ttc(d, ve, 0, 0, 0):.2f} s")
    print(f"2. (Ego 20, Target 10):     {calculate_universal_ttc(d, ve, 0, 10.0, 0):.2f} s")
    print(f"3. (Ego gyorsul, Target 20):     {calculate_universal_ttc(d, ve, 5.0, 10.0, 0):.2f} s")
    print(f"4. (Ego 20, Target lassul):  {calculate_universal_ttc(d, ve, 0, 10.0, -5.0):.2f} s")
    print(f"5. (Mindkettő fékez):       {calculate_universal_ttc(d, ve, -2.0, 10.0, -5.0):.2f} s")
    print(f"6. (Ego 20, Target 20 (nincs ütközés)):      {calculate_universal_ttc(d, ve, 0, 20.0, 0):.2f} s")

