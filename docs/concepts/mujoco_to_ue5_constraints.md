# Конвертация MuJoCo joints в UE5 Physics Constraints

Эта страница описывает алгоритм экспорта кинематического дерева MuJoCo в
`UPhysicsConstraintComponent`/Chaos. Она нужна только если физика должна быть
перенесена из MuJoCo в Unreal. В обычном режиме URLab **не должен** создавать
Chaos-констрейны: MuJoCo остаётся единственным владельцем физического состояния,
а Unreal получает только render snapshot. Одновременный `mj_step` и
`SetSimulatePhysics(true)` для тех же тел приводит к двойной интеграции и
нестабильности.

## 1. Зафиксировать модель и единицы

Нельзя конвертировать joint только по исходной строке MJCF. Сначала нужно
получить скомпилированную модель с учётом `<default>`, `childclass`,
`compiler angle`, `ref`, `qpos0` и всех локальных body-трансформаций.

Рекомендуемый источник данных:

1. создать `mjModel` и `mjData` через обычный MuJoCo compile pipeline;
2. записать `mjData.qpos = mjModel.qpos0`;
3. вызвать `mj_forward`;
4. читать `xpos`, `xquat`, `xanchor`, `xaxis`, `jnt_type`, `jnt_range`,
   `jnt_qposadr` и `jnt_dofadr` из этой согласованной reference pose.

У MuJoCo joint принадлежит body, в котором он объявлен. `joint.pos` и
`joint.axis` заданы в frame этого body, а сам joint задаёт движение body
относительно его родителя. Это важно: не следует трактовать `pos` как позицию
в frame родительского body.

Для URLab используются следующие преобразования (MuJoCo: RHS, метры, Z-up;
Unreal: сантиметры, координатная конвенция URLab):

```text
P_ue = (100 * P_m.x, -100 * P_m.y, 100 * P_m.z)
D_ue = (D_m.x,       -D_m.y,       D_m.z)
```

Для кватерниона MuJoCo `[w, x, y, z]` результат для `FQuat(x, y, z, w)`:

```text
Q_ue = (-x_m, y_m, -z_m, w_m)
```

Кватернион и ось после преобразования нужно нормализовать; нулевая или
нечисловая ось является ошибкой импорта. В исходном коде URLab эти правила
централизованы в `MjUtils` и `URLabAxisConv` — конвертер не должен вводить
второй, слегка отличающийся набор swizzle-правил.

## 2. Создать физические тела

Для каждого MuJoCo body создать ровно одно UE physics body (обычно
`UPrimitiveComponent` с `UBodySetup`, либо body в Physics Asset). Не использовать
иерархию `AttachToComponent` как замену joint: attachment может принудительно
переписывать transform, тогда как constraint должен быть единственным
механизмом связи тел.

Для каждого body:

- поставить reference world transform, рассчитанный MuJoCo и переведённый в UE;
- перенести mass без изменения, COM из m в cm, а inertia с учётом масштаба
  `100²` и Y-reflection (`I_ue = 100² * C * I_m * Cᵀ`, где
  `C = diag(1, -1, 1)`);
- перенести collision shapes и их фильтры отдельно от joint;
- для worldbody использовать static world, а не динамическое тело;
- дождаться создания physics state, но не включать симуляцию до создания всех
  constraints.

Если MuJoCo body не имеет joint, он welded к родителю. Его нельзя автоматически
считать свободным только потому, что у него есть collision mesh.

## 3. Вычислить frame joint в world space

Для каждого joint получить из reference pose:

```text
A_m^W = xaxis[joint]      # мировая единичная ось MuJoCo
P_m^W = xanchor[joint]    # мировая точка шарнира, метры
```

Если данные вычисляются вручную, эквивалентная формула для body, в котором
объявлен joint, имеет вид:

```text
P_m^W = body_xpos + body_xmat * joint_pos
A_m^W = normalize(body_xmat * joint_axis)
```

Перевести их в UE:

```text
P_ue^W = MjPositionToUe(P_m^W)
A_ue^W = normalize(MjDirectionToUe(A_m^W))
```

Далее построить ортонормированный joint frame `J^W`:

- локальная ось **X** должна совпадать с `A_ue^W`;
- для Hinge X — ось вращения, для Slide X — направление перемещения;
- вторичную ось выбирать детерминированно (`Up`, а если ось почти параллельна
  `Up`, то `Forward`), используя штатный `FRotationMatrix::MakeFromX/MakeFromXZ`;
- не менять знак X случайно: он определяет знак положительного qpos и actuator
  gear;
- для Ball и Free roll frame не влияет на DOF, но должен быть стабильным между
  импортами.

Положение и ориентация frame в каждом из двух body выражаются отдельно:

```text
FrameParent = J^W.GetRelativeTransform(ParentBody^W)
FrameChild  = J^W.GetRelativeTransform(ChildBody^W)
```

Оба frame должны описывать одну и ту же мировую точку и одну и ту же мировую
ориентацию. Нельзя ставить оба frame в `(0, 0, 0)` только потому, что joint
компонент находится в начале UE-компонента.

## 4. Создать UE constraint

Создать один `UPhysicsConstraintComponent` на связь parent/child, выставить его
world transform в `J^W`, вызвать `SetConstrainedComponents`, а затем записать
`FrameParent` и `FrameChild` в `FConstraintInstance` (через API версии UE,
который устанавливает `Frame1`/`Frame2`). До запуска Chaos полезно проверить,
что оба frame совпадают с `J^W` с точностью до допуска.

Для обычного дерева parent — это родительский MuJoCo body, а child — body,
содержащий joint. Для joint у body первого уровня вместо фиктивного
динамического parent использовать world/static body. Если текущая версия UE
не принимает `nullptr` как world component, создать один неподвижный anchor
component.

Соседние body не следует безусловно исключать из collision. Решение о collision
должно следовать `contype`, `conaffinity`, contact pairs и exclusions MuJoCo;
`DisableCollision` включается только для конкретной пары, если это соответствует
исходной модели.

## 5. Таблица степеней свободы

Перед настройкой каждое движение нужно явно залочить. Нельзя полагаться на
профиль UE по умолчанию.

| MuJoCo joint | UE linear motion | UE angular motion | Ограничение |
|---|---|---|---|
| `hinge` | X/Y/Z locked | Swing1/Swing2 locked, Twist свободен или limited | Twist вокруг frame X |
| `slide` | X свободен или limited, Y/Z locked | Twist/Swing1/Swing2 locked | Linear X вдоль frame X |
| `ball` | X/Y/Z locked | Twist/Swing1/Swing2 свободны | 3 вращательных DOF |
| `free` | X/Y/Z свободны | Twist/Swing1/Swing2 свободны | обычно constraint не нужен |
| body без joint | X/Y/Z locked | Twist/Swing1/Swing2 locked | weld/fixed |

В Chaos стандартная интерпретация constraint frame использует X как twist и
как Linear X. Это нужно подтвердить маленьким axis-probe на целевой версии UE:
положительное вращение/перемещение тестового body должно совпадать с
положительным MuJoCo qpos. Если знак противоположный, исправлять его нужно
согласованно (frame axis **или** actuator gear), а не Y-flip-ом в случайном
месте.

`free` joint в MuJoCo обычно разрешён только у body первого уровня и даёт
body шесть DOF относительно мира. Такой body следует оставить динамическим без
ограничивающего constraint; создание «свободного» constraint с якорем не даёт
дополнительной точности.

## 6. Перенос range и reference pose

Для Hinge значения MuJoCo — углы, для Slide — расстояния. После compile
использовать радианы и метры, затем переводить в UE API:

```text
hinge: limitDeg = RadToDeg(limitRad)
slide: limitCm  = 100 * limitMeters
```

`range = [lo, hi]` — это диапазон qpos, а не обязательно диапазон вокруг
текущего transform. Для scalar joint сначала вычислить reference value
`q0 = qpos0[qposadr]` и рабочий displacement:

```text
d_min = lo - q0
d_max = hi - q0
```

Встроенные UE limits во многих версиях задаются симметрично относительно нуля.
Если `d_min = -d_max`, можно передать половину диапазона напрямую. Если диапазон
асимметричен, нужен один из вариантов:

1. API/профиль Chaos с явным reference offset — центрировать диапазон в
   `d_mid = (d_min + d_max) / 2` и задать half-width
   `(d_max - d_min) / 2`;
2. собственный limit drive/constraint, который проверяет qpos относительно
   `[lo, hi]`;
3. дополнительный proxy body и явный механический offset.

Нельзя «исправить» асимметрию перемещением только одного anchor frame: frames
перестанут совпадать в reference pose и constraint создаст начальный импульс.

`ref` не нужно механически копировать в `AngularRotationOffset`. Сначала
зафиксировать геометрическую reference pose, а значение joint хранить отдельно
для контроллера и телеметрии. При измерении scalar joint использовать
нормализованную signed delta относительно `q0`, с обработкой перехода угла
через `+/-pi`.

Для Ball использовать quaternion/qpos из MuJoCo, а не пытаться представить его
одним Euler-углом. Для Free использовать 3 позиции и 4-компонентный quaternion;
не применять к нему hinge/ball limits.

## 7. Spring, damping и actuator

Это перенос поведения, а не простое копирование чисел:

- `stiffness` и `damping` преобразовать в UE linear/angular drive либо в
  собственный PD/torque controller;
- для Hinge использовать физическое уравнение
  `tau = -k * (theta - theta_s) - c * omega`, где `theta` в радианах;
- для Slide использовать
  `F = -k * (x - x_s) - c * v`, где `x` в метрах;
- на границе Unreal учитывать cm и угловые единицы именно той функции Chaos,
  которая вызывается — нельзя напрямую копировать коэффициенты из MuJoCo;
- `armature` — это reflected inertia joint, у UE constraint нет универсального
  эквивалента; его добавляют в inertia/motor model, а не в mass без расчёта;
- `frictionloss`, `solref`, `solimp` и `margin` не имеют точного 1:1 отображения
  в `FConstraintInstance`; их нужно реализовать отдельным friction/limit
  controller и калибровать по тесту;
- actuator — это источник силы/момента, не ещё один joint constraint. `gear`,
  ctrlrange и forcerange надо переносить в motor/drive и проверять в единицах
  Chaos.

Если требуется воспроизводимость MuJoCo, оставлять actuator и solver в MuJoCo.
UE constraint export подходит для визуализации, интерактивной Chaos-физики или
приближённой динамики, но не является математически эквивалентной заменой
MuJoCo solver.

## 8. Несколько joints на одном body

MuJoCo допускает несколько joints в одном body; их spatial transforms применяются
последовательно. Один UE `FConstraintInstance` корректно описывает только один
совместимый набор DOF в одном frame. Простое суммирование осей ломается, если
joints имеют разные `pos`, непараллельные оси или порядок важен.

Алгоритм:

1. если у body один hinge/slide/ball — создать один constraint;
2. если несколько joints совпадают в frame и их DOF точно представимы профилем
   Chaos — явно доказать это axis/pose тестом и агрегировать;
3. иначе разложить body на последовательность proxy bodies/frames, по одному
   вращательному или поступательному joint на constraint, сохранив порядок MJCF;
4. collision и визуальный mesh оставить на физически осмысленном proxy, массу и
   inertia не дублировать; massless proxy не использовать без понимания
   устойчивости solver;
5. для ball + slide и других смешанных случаев предпочесть proxy chain, если
   axes/anchors не совпадают.

Нельзя накладывать несколько независимых constraints с разными frame прямо на
одну пару тех же двух тел: это создаёт лишние уравнения и обычно overconstraint.

## 9. Проверка конвертации

Конвертер должен выдавать отчёт для каждого joint (`name`, parent, child, type,
axis, anchor, q0, range, sign). Минимальный набор автоматических проверок:

1. **Pose:** parent и child body в reference pose совпадают с MuJoCo по позиции,
   quaternion и COM; anchor error меньше заданного допуска.
2. **Frame:** `dot(A_mapped, A_ue) > 0.999`, frame orthonormal, finite, без
   отрицательного scale.
3. **DOF:** малое положительное изменение qpos разрешает только нужное движение;
   все locked coordinates остаются ниже допуска.
4. **Sign:** `qpos += epsilon` перемещает/вращает child по `A_ue`, а не против
   неё.
5. **Limits:** проверить обе границы, значение внутри и значение за каждой
   границей; отдельно проверить асимметричный диапазон.
6. **Dynamics:** подать один импульс и сравнить знак, порядок величины и
   trajectory joint с MuJoCo. Совпадение до последнего бита не ожидается из-за
   разных solvers.
7. **Startup:** после создания constraints нет начального взрыва, NaN,
   penetration impulse или заметного смещения из reference pose.

## Псевдокод

```text
model, data = CompileMJCF(xml)
data.qpos = model.qpos0
mj_forward(model, data)

for body in model.bodies:
    ue_body[body] = CreatePhysicsBody(
        transform = ConvertBodyPose(data.xpos[body], data.xquat[body]),
        mass_com_inertia = ConvertInertial(model, body),
        collision = ConvertGeoms(body))

for body in model.bodies excluding world:
    joints = OrderedJointsOf(body)

    if joints is empty:
        CreateFixedConstraint(ue_body[parent(body)], ue_body[body])
        continue

    groups = SplitIntoCompatibleConstraintFrames(joints)
    for group in groups:
        anchor_m = data.xanchor[group.first]
        axis_m   = data.xaxis[group.first]
        J_ue     = MakeJointFrame(ConvertPosition(anchor_m),
                                   ConvertDirection(axis_m))

        c = CreateConstraint()
        c.SetConstrainedComponents(ue_body[parent(group)], ue_body[child(group)])
        c.SetFrames(Relative(J_ue, ue_body[parent(group)]),
                    Relative(J_ue, ue_body[child(group)]))
        c.SetAllMotionsLocked()
        ConfigureDOF(c, group.type)
        ConfigureLimitsFromQposRange(c, group, model.qpos0)
        ConfigureDriveFromPhysicalParameters(c, group)

ValidateReferencePoseAndDOF()
EnableChaosSimulationOnlyAfterValidation()
```

## Что делать в URLab

Текущий URLab уже реализует правильную для симулятора схему: `UMjBody` и
`UMjJoint` регистрируют MJCF в `mjSpec`, `UMjPhysicsEngine` компилирует модель,
а `UMjBody::ApplyRenderState` применяет согласованный render snapshot. Поэтому
для обычного импорта MJCF не нужно добавлять `UPhysicsConstraintComponent`.

Если нужен отдельный экспорт в UE/Chaos, его лучше сделать отдельным режимом
или отдельным editor commandlet с явным выбором physics owner. При включённом
экспорте нужно отключить MuJoCo stepping для этих тел; при включённом MuJoCo
режиме constraints должны оставаться только описанием/визуализацией и не должны
симулироваться Chaos.
