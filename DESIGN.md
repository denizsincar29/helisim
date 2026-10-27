# helisim — модуль вертолёта

Как пользоваться: см. `README.md` и `examples/`. Файлы: `helisim.py` (модуль), `test_features.py` (moved, пауза, край карты, футы), `demo_flight.py` (запуск→взлёт→полёт→посадка для 3 пресетов),
`test_scenarios.py` (отказы, столкновения, отклонённые команды).

## Как игра общается с модулем

```python
heli = Helicopter("medium_turbine", timestamp=t0, x=0, y=0, z=ground)   # или HelicopterConfig(preset, **overrides)
heli.switch("battery", True)        # тумблеры: battery generator fuel_pump fuel_valve ignition* avionics
heli.toggle("rotor_brake")          #   nav_lights landing_light anti_collision rotor_brake clutch* sas
heli.button("starter")              # кнопки: starter (сам отсечётся по оборотам), engine_stop
heli.turn("collective", 0.6)        # оси: collective, throttle (0..1); cyclic_roll, cyclic_pitch, pedals (-1..1)
heli.step("pedals", -0.1)           # сдвиг на delta (для клавиатуры)
heli.joystick(roll, pitch)          # циклик одним вызовом
events = heli.update(timestamp)     # каждые X секунд; список Event(t, name, data)
heli.get_state()                    # dict для звука / озвучки по запросу
heli.controls_info()                # список органов с подписями - для меню
```
`*` - только если есть у вертолёта (ignition/clutch - у поршневого).
Положение осей идёт к цели с ограниченной скоростью (как рука пилота): `axis` - реальное, `targets` - заданное.

### Сообщения от игры
`reached_surface(ground_z)`, `reached_wall(nx, ny, part)` (`part`: fuselage/rotor/tail),
`reached_ceiling()`, `reached_water()`, `surface_lost()`, `set_ground_level(z)`,
либо единый `collision(kind, ...)`. Исход решает модуль по скорости/углу: `touchdown`, `bump` или `crashed`
(`crashed=True`, причина в `crash_reason`: hard_landing, tilted_landing, fast_landing, wall_impact,
rotor_strike, tail_strike, water, rotor_overspeed, structural_failure).
`auto_ground=True` - вертолёт сам ловит z<=ground_z (для тестов без игры).

## События
Дискретные: `switch`, `button`, `command_rejected{name,reason}`, `paused`, `resumed`, `map_edge`, `engine_state{old,new}`, `engine_started`,
`starter_cutout`, `start_failed`, `hot_start`, `engine_flameout`, `engine_failure`, `fuel_exhausted`,
`liftoff`, `touchdown`, `left_surface`, `bump`, `crashed`, `bus_power_changed`.

Предупреждения: `warning{id, active}`; id: low_rotor_rpm, high_rotor_rpm, overtorque, overtemp, low_fuel,
low_battery, vortex_ring, retreating_blade_stall, overspeed, rotor_brake_on, hard_landing.

Изменения (по порогам из конфига, чтобы не спамить): `rotor_rpm_changed`, `engine_rpm_changed`,
`torque_changed`, `altitude_changed`, `moved`, `heading_changed`, `attitude_changed`,
`speed_changed`, `vertical_speed_changed`, `vertical_trend_changed{climb|descent|level}`, `fuel_changed`.

## Что смоделировано
- Электрика: батарея, генератор, нагрузки, просадка при старте. Без питания стартёр отклоняется.
- Двигатель: off → cranking → starting → running → shutdown/failed; турбина (топливный кран + насос +
  стартёр, зависание запуска, горячий запуск) и поршень (магнето + сцепление). EGT, перегрев съедает ресурс.
- Ротор: энергетический баланс (инерция НВ), регулятор оборотов с обгонной муфтой, тормоз ротора.
  Из-за муфты авторотация получается сама, без отдельного кода.
- Тяга от шага и оборотов, плотность по высоте (ISA), эффект земли, индуктивная скорость по теории
  импульса (подъём/снижение/поступательная скорость - Glauert, Johnson), профильная мощность растёт со скоростью.
- Вихревое кольцо (снижение с мощностью на малой скорости), срыв на отступающей лопасти
  (скорость и перегрузка), низкие/высокие обороты, Vne.
- Реактивный момент от мощности, хвостовой винт с нейтралью под висение (на педалях нужна подстройка
  при смене шага), демпфирование, флюгерность.
- Расход топлива от мощности, масса меняется.

## Конфиг и пресеты
`light_piston` (R22-подобный), `medium_turbine` (Bell 206-подобный), `heavy_twin` (Ми-8-подобный, ротор по часовой).
Любой параметр переопределяется: `HelicopterConfig("medium_turbine", mtow=1600, idle_nr=0.6, crash_sink=4.0)`.
Ключи - в `_DEFAULTS` в начале `helisim.py`. Режим циклика: `cyclic_mode="attitude"` (стик = угол, удобно с
клавиатуры) или `"rate"` (стик = скорость вращения, ближе к реальности).

## Позиция: вертолёт не знает карту
По умолчанию `position_mode="delta"`: абсолютных координат в API нет (ни в `get_state()`, ни в событиях).
Вместо них событие **`moved`** со смещением с прошлого отчёта:
`dx, dy` (восток/север), `dz`, `distance` (горизонтальная), `bearing` (компасный курс движения, 0 = север),
`relative_bearing` (направление движения относительно носа, -180..180), `elevation` (угол подъёма/спуска, град),
`heading` (куда смотрит нос). Игра сама двигает игрока: `player.x += dx` или "пройти distance под углом relative_bearing".
`z` (высота над поверхностью, плотность воздуха) вертолёт держит сам; `dz` в `moved` - для синхронизации.
`move_step` (метры): копить смещение и слать реже; смещение не теряется. `position_mode="absolute"` возвращает
`x, y` в `get_state()` и событие `position_changed` (для тестов/отладки).

```python
for e in heli.update(now):
    if e.name == "moved":
        player.move(e.data["dx"], e.data["dy"]); player.z += e.data["dz"]
    elif e.name == "crashed": ...
```

### Край карты
`heli.reached_map_edge(nx=None, ny=None, mode=None)` - вертолёт карту не знает и о переходе на другую карту ему
знать не нужно. `mode="pass"` (по умолчанию, `map_edge_mode` в конфиге): физика не меняется, только событие `map_edge`
(звук/озвучка), игра сама меняет карту и позицию (после смены - `set_ground_level` для новой поверхности).
`mode="block"`: гасит скорость в сторону края (нормаль (nx, ny) смотрит внутрь карты; без нормали - всю горизонтальную),
звать каждый кадр, пока упираемся. Событие `map_edge` не чаще раза в секунду.

## Пауза
`pause(reason)` / `resume(timestamp=None)`: на паузе `update()` ничего не считает (команды принимаются и копятся в
событиях), время симуляции стоит, таймеры кнопок тоже. После `resume()` часы перепривязываются на ближайшем `update()`
(или сразу, если передан `timestamp`), поэтому долгая загрузка карты не превращается в скачок физики. Работает и с
системным таймером (`update()` без аргумента), и с вашими таймстампами. `reset_clock()` - перепривязка без паузы.
`max_dt` (1 с) - только защита от одиночного лага, паузу он не заменяет.

## Единицы
Внутри всегда СИ. `imperial_units=True` (в конфиге или `Helicopter(..., imperial_units=True)`) переключает границу API:
позиции/высоты в футах, скорости в фут/с. Это касается входов (`x,y,z,ground_z` в конструкторе,
`reached_surface(ground_z)`, `set_ground_level`, `set_wind`) и выходов (`get_state()`, данные событий:
x y z agl dx dy dz distance vz ground_speed airspeed speed sink). `get_state()["units"]` говорит, в чём сейчас значения.
Масса/топливо остаются в кг; углы в градусах. Публичные атрибуты `heli.x/.z/.vx` - всегда метры (внутренние).

## Тюнинг (результат замера `measure.py`, перемеряет все пресеты)
| пресет | шаг на висении (50 м, полные баки) | мощность на висении | полный стик / Vne | пик набора | потолок |
|---|---|---|---|---|---|
| light_piston | 0.56 | 56% | 1.05 | 8 м/с | ~3500 м |
| medium_turbine | 0.60 | 55% | 0.98 | 10 м/с | ~2000 м |
| medium_turbine_agile (прежняя настройка) | 0.53 | 46% | 0.95 | 14 м/с | ~3500 м |
| heavy_twin | 0.52 | 54% | 0.93 | 12 м/с | ~3500 м |

Авторотация (шаг подбирается по оборотам): снижение 5-7.5 м/с, обороты ~100%.
Ручки: `thrust_ratio` (запас тяги), `drivetrain_eff` (КПД привода), `kappa`/`profile_power_kw` (потребная мощность),
`heave_damp` (демпфирование вертикальной скорости), `cda_h` (скорость на полном стике), `tail_*`/`yaw_damp` (педали),
`nr_overspeed` (порог разрушения ротора от раскрутки). Новый пресет - новая запись в `PRESETS` (только отличия от `_DEFAULTS`).

## Известные упрощения / дальше
- Ветер: поле `wind` уже участвует во всех расчётах скорости относительно воздуха - потом просто задавать.
- Нет: динамического опрокидывания на земле, отказа хвостового винта, поломок по частям, температуры воздуха, лётных ограничений по массе/центровке.
- Тюнинг: числа подобраны так, чтобы поведение было похоже на прототипы, но не сверялись с РЛЭ.

## Перенос на NVGT
Код намеренно плоский: простые классы, `dict`↔`dictionary`, float, массивы, без dataclass/генераторов/лямбд.
`Event.data` - dictionary<string, variant>; `update()` возвращает массив событий (колбэки `subscribe` можно не переносить).
`time.monotonic()` заменить на `ticks()/1000.0`. Конструктор конфига с `**overrides` заменить на dictionary-переопределения.
