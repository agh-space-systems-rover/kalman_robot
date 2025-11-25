# Kalman Photo App

Aplikacja do przeglądania zdjęć z danymi trajektorii, pozwalająca na transformację współrzędnych lokalnych obiektów na współrzędne globalne wykrytych cubów.

## Opis

Kalman Photo App umożliwia:
- Przeglądanie zdjęć z katalogu
- Wyświetlanie informacji o obiektach wykrytych na zdjęciach
- Transformację współrzędnych lokalnych obiektów na współrzędne globalne
- Zapisywanie przetransformowanych współrzędnych do plików

## Instalacja

1. Sklonuj repozytorium lub pobierz kod źródłowy

2. Zainstaluj wymagane zależności:
    ```bash
    pip install -r requirements.txt
    ```

## Uruchomienie

Uruchom aplikację komendą:

```bash
python main.py <folder_path> <trajectory_file.yaml> <out_folder>
```

gdzie:
- `<folder_path>` - ścieżka do katalogu ze zdjęciami
- `<trajectory_file.yaml>` - plik YAML zawierający dane trajektorii
- `<out_folder>` - folder, do którego zostaną zapisane wynikowe pliki

## Przyciski i sterowanie

| Przycisk | Akcja |
|----------|-------|
| `←` (strzałka w lewo) | Poprzednie zdjęcie |
| `→` (strzałka w prawo) | Następne zdjęcie |
| `1`-`9` | Pokazuje popup z informacjami o wybranym obiekcie |
| `Esc` | Zamyka popup |
| `Spacja` | Zapisuje współrzędne globalne wybranego obiektu do pliku |

## Struktura plików

- `main.py` - główny plik aplikacji
- `gui/image_viewer.py` - zawiera klasę do wyświetlania zdjęć
- `utils/` - katalog z narzędziami pomocniczymi:
  - `trajectory.py` - klasa do obsługi trajektorii
  - `timestamp.py` - funkcje do obsługi znaczników czasu
  - `image_loader.py` - funkcje do ładowania obrazów
  - `position_calculate.py` - funkcje do transformacji współrzędnych

## Format danych

Aplikacja oczekuje:
- Obrazów w popularnych formatach (.jpg, .jpeg, .png, .gif, .bmp)
- Plików tekstowych z opisem obiektów (tej samej nazwy co zdjęcia, ale z rozszerzeniem .txt)
- Pliku trajektorii w formacie YAML

## Przepływ pracy

1. Uruchom aplikację podając odpowiednie parametry
2. Przeglądaj zdjęcia używając strzałek
3. Kliknij numer obiektu aby zobaczyć jego szczegóły
4. Naciśnij spację aby zapisać przetransformowane współrzędne do pliku