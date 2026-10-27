# helisim — модуль вертолёта

Файлы: `helisim.py` (модуль), `demo_flight.py` (запуск→взлёт→полёт→посадка для 3 пресетов),
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
Дискретные: `switch`, `button`, `command_rejected{name,reason}`, `engine_state{old,new}`, `engine_started`,
`starter_cutout`, `start_failed`, `hot_start`, `engine_flameout`, `engine_failure`, `fuel_exhausted`,
`liftoff`, `touchdown`, `left_surface`, `bump`, `crashed`, `bus_power_changed`.

Предупреждения: `warning{id, active}`; id: low_rotor_rpm, high_rotor_rpm, overtorque, overtemp, low_fuel,
low_battery, vortex_ring, retreating_blade_stall, overspeed, rotor_brake_on, hard_landing.

Изменения (по порогам из конфига, чтобы не спамить): `rotor_rpm_changed`, `engine_rpm_changed`,
`torque_changed`, `altitude_changed`, `position_changed`, `heading_changed`, `attitude_changed`,
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

## Известные упрощения / дальше
- Ветер: поле `wind` уже участвует во всех расчётах скорости относительно воздуха - потом просто задавать.
- Нет: динамического опрокидывания на земле, отказа хвостового винта, поломок по частям, температуры воздуха, лётных ограничений по массе/центровке.
- Тюнинг: числа подобраны так, чтобы поведение было похоже на прототипы, но не сверялись с РЛЭ.

## Перенос на NVGT
Код намеренно плоский: простые классы, `dict`↔`dictionary`, float, массивы, без dataclass/генераторов/лямбд.
`Event.data` - dictionary<string, variant>; `update()` возвращает массив событий (колбэки `subscribe` можно не переносить).
`time.monotonic()` заменить на `ticks()/1000.0`. Конструктор конфига с `**overrides` заменить на dictionary-переопределения.
