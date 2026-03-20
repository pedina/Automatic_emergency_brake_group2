#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iomanip>

/**
 * Kiszámítja az univerzális ütközésig hátralévő időt (TTC) a relatív mozgás alapján.
 * * Paraméterek:
 * - distance (double): A kezdeti távolság az ego és a target jármű között (méter, > 0).
 * - v_ego, v_target (double): Az ego és a target sebessége (m/s).
 * - a_ego, a_target (double): Az ego és a target gyorsulása (m/s²).
 * - accel_threshold (double): Küszöbérték, amely alatt a relatív gyorsulást elhanyagolhatónak (konstans sebességnek) tekintjük.
 * * Visszatérési érték:
 * - double: A TTC értéke másodpercben, vagy végtelen (inf), ha nem várható ütközés.
 * * Képlet: A 0.5 * a_rel * t² + v_rel * t - distance = 0 másodfokú egyenletet oldja meg, feltéve hogy a_rel != 0.
 */
double calculate_universal_ttc(double distance, double v_ego, double a_ego, double v_target, double a_target, double accel_threshold = 1e-5) {
    if (distance <= 0) {
        return 0.0; // Már megtörtént az ütközés
    }

    double rel_velocity = v_ego - v_target;
    double rel_acceleration = a_ego - a_target;

    // 1. ESET: Nincs (vagy elhanyagolható) relatív gyorsulás
    if (std::abs(rel_acceleration) < accel_threshold) {
        if (rel_velocity <= 0) {
            return std::numeric_limits<double>::infinity(); // Távolodunk vagy tartjuk a távolságot
        }
        return distance / rel_velocity;
    }

    // 2. ESET: Van relatív gyorsulás (másodfokú egyenlet)
    // Diszkrimináns: b^2 - 4ac -> rel_velocity^2 - 4 * (0.5 * rel_acceleration) * (-distance)
    double discriminant = std::pow(rel_velocity, 2) + 2 * rel_acceleration * distance;

    if (discriminant < 0) {
        return std::numeric_limits<double>::infinity(); // Ha a diszkrimináns negatív, nincs ütközés
    }

    // Két lehetséges időpont kiszámítása
    double sqrt_disc = std::sqrt(discriminant);
    double t1 = (-rel_velocity + sqrt_disc) / rel_acceleration;
    double t2 = (-rel_velocity - sqrt_disc) / rel_acceleration;

    // Csak a jövőbeli (pozitív) időpontok számítanak
    double min_valid_time = std::numeric_limits<double>::infinity();
    
    if (t1 > 0) {
        min_valid_time = std::min(min_valid_time, t1);
    }
    if (t2 > 0) {
        min_valid_time = std::min(min_valid_time, t2);
    }

    return min_valid_time;
}

int main() {
    double d = 50.0;  
    double ve = 20.0; 

    // Beállítjuk, hogy a konzol mindig 2 tizedesjeggyel írja ki a lebegőpontos számokat
    std::cout << std::fixed << std::setprecision(2);

    std::cout << "1. (Ego 20, Target áll):    " << calculate_universal_ttc(d, ve, 0, 0, 0) << " s\n";
    std::cout << "2. (Ego 20, Target 10):     " << calculate_universal_ttc(d, ve, 0, 10.0, 0) << " s\n";
    std::cout << "3. (Ego gyorsul, Target 20):" << calculate_universal_ttc(d, ve, 5.0, 10.0, 0) << " s\n";
    std::cout << "4. (Ego 20, Target lassul): " << calculate_universal_ttc(d, ve, 0, 10.0, -5.0) << " s\n";
    std::cout << "5. (Mindkettő fékez):       " << calculate_universal_ttc(d, ve, -2.0, 10.0, -5.0) << " s\n";
    std::cout << "6. (Ego 20, Target 20 (nincs ütközés)): " << calculate_universal_ttc(d, ve, 0, 20.0, 0) << " s\n";

    return 0;
}