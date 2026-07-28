<div dir="rtl">

# مرجع API

مرجع کامل همهٔ خروجی‌های عمومی Pulse2D.

تایپ‌ها به‌صورت TypeScript نشان داده شده‌اند. `Scalar` نوع عددی بک‌اند فعال است —
در هر دو نسخه `number` است، ولی با آن مثل یک نوع مبهم رفتار کنید و به‌جای
محاسبات خام، صریحاً با `Scalar.fromFloat` / `Scalar.toFloat` تبدیل کنید.

[English version](../API.md)

**فهرست**

- [World](#world) · [Body](#body) · [Fixture](#fixture) · [شکل‌ها](#شکلها)
- [مفصل‌ها](#مفصلها) · [تماس و رویداد](#تماس-و-رویداد) · [فیلترینگ](#فیلترینگ)
- [ریاضیات](#ریاضیات) · [شبکه](#شبکه) · [رندر دیباگ](#رندر-دیباگ)
- [درون فاز برخورد](#درون-فاز-برخورد) · [تنظیمات](#تنظیمات)

---

## World

ظرف شبیه‌سازی. مالک اجسام، فیکسچرها، تماس‌ها، مفصل‌ها، فاز گسترده و حل‌کننده.

### `new World(def?: WorldDef)`

</div>

```ts
interface WorldDef {
  gravity?: { x: number; y: number };  // پیش‌فرض (0, -10) متر بر مجذور ثانیه
  timeStep?: number;                   // پیش‌فرض 1/60 ثانیه — برای همیشه ثابت
  subSteps?: number;                   // پیش‌فرض 4
  velocityIterations?: number;         // پیش‌فرض 2، در هر گام فرعی
  relaxIterations?: number;            // پیش‌فرض 1، در هر گام فرعی
  enableSleep?: boolean;               // پیش‌فرض true
  enableWarmStarting?: boolean;        // پیش‌فرض true (فقط برای دیباگ خاموش کنید)
  enableRestitution?: boolean;         // پیش‌فرض true
  seed?: number;                       // seed برای world.rng
}
```

<div dir="rtl">

پارامتر `subSteps` اهرم اصلی کیفیت/هزینه است: گام‌های فرعی بیشتر یعنی پشته‌های
سفت‌تر و مدیریت بهتر حرکت سریع، با هزینهٔ خطی. ترجیحاً این را بالا ببرید نه
`velocityIterations` را.

### گام برداشتن

| عضو | توضیح |
|---|---|
| `step(): void` | دقیقاً یک گام ثابت جلو می‌رود. قطعی. |
| `accumulate(dt: number, maxSteps?: number): number` | از زمان فریم متغیر، گام‌های کامل اجرا می‌کند. کسر باقیمانده در `[0,1)` را برای درون‌یابی رندر برمی‌گرداند. `maxSteps` (پیش‌فرض `۵`) سقف جبران است. |
| `tick: number` | تعداد گام‌های سپری‌شده. شمارندهٔ tick برای lockstep. |
| `time: Scalar` | کل زمان شبیه‌سازی‌شده. |
| `timeStep: Scalar` | مدت گام (فقط خواندنی). |
| `invSubStep: Scalar` | `1 / (timeStep / subSteps)`. ضربه‌های مفصل را به نیرو تبدیل می‌کند. |

### اجسام و مفصل‌ها

| عضو | توضیح |
|---|---|
| `createBody(def?: BodyDef): Body` | ساخت جسم. |
| `destroyBody(body: Body): void` | حذف جسم به‌همراه فیکسچرها، تماس‌ها و مفصل‌هایش. |
| `createRevoluteJoint(def)` / `createRevoluteJointAt(bodyA, bodyB, x, y, extra?)` | لولا. شکل `At` لنگرگاه‌های محلی را از یک نقطهٔ جهانی حساب می‌کند. |
| `createPrismaticJoint(def)` | ریل کشویی. |
| `createDistanceJoint(def)` | فاصله / فنر / طناب. |
| `createWeldJoint(def)` | جوش صلب یا نرم. |
| `createMouseJoint(def)` | کشیدن به سمت یک هدف. |
| `createMotorJoint(def)` | رساندن به آفست هدف. |
| `destroyJoint(joint: Joint): void` | حذف مفصل. |

### کوئری‌ها

</div>

```ts
rayCastClosest(x1, y1, x2, y2, filter?): { fixture, point, normal, fraction } | null
```

<div dir="rtl">

نزدیک‌ترین برخورد یا `null`. پارامتر `filter` می‌تواند فیکسچرها را قبل از بررسی
رد کند.

</div>

```ts
rayCast(x1, y1, x2, y2, cb: (fixture, point, normal, fraction) => Scalar): void
```

<div dir="rtl">

همهٔ برخوردها. مقدار بازگشتی callback پیمایش را هدایت می‌کند:

| بازگشت | معنی |
|---|---|
| `-1` | این فیکسچر را نادیده بگیر، با بازهٔ فعلی ادامه بده |
| `0` | همین حالا متوقف شو |
| `fraction` | ادامه بده ولی فقط برخوردهای نزدیک‌تر را گزارش کن |
| `1` | با بازهٔ کامل ادامه بده |

</div>

```ts
queryAABB(lowerX, lowerY, upperX, upperY, cb: (fixture) => boolean): void
queryPoint(x, y, cb: (fixture) => boolean): void
```

<div dir="rtl">

برای توقف زودهنگام از callback مقدار `false` برگردانید. متد `queryPoint`
دربرگیری واقعی شکل را تست می‌کند، نه فقط AABB را.

### بازرسی

| عضو | توضیح |
|---|---|
| `bodyCount` / `awakeBodyCount` / `contactCount` / `jointCount` | شمارنده‌ها. |
| `eachBody(): IterableIterator<Body>` | اجسام زنده به ترتیب شناسه. |
| `eachJoint(): IterableIterator<Joint>` | مفصل‌های زنده به ترتیب شناسه. |
| `contacts: Contact[]` | تماس‌های زنده، به ترتیب متعارف حل. |
| `rng: Rng` | مولد تصادف seed‌دار دنیا — به‌جای `Math.random` از این استفاده کنید. |
| `profile` | زمان‌بندی هر گام به میلی‌ثانیه: `total`، `broadPhase`، `narrowPhase`، `solve`، `continuous` و شمارنده‌ها. |
| `gravity: Vec2` | قابل تغییر، ولی باید بین همتاها یکسان باشد. |

### سایر

| عضو | توضیح |
|---|---|
| `setListener(l: WorldListener \| null)` | نصب callbackهای رویداد. |
| `wakeAll()` | بیدار کردن همهٔ اجسام. |
| `clear()` | پاک کردن همه‌چیز؛ دنیا قابل استفادهٔ مجدد می‌ماند. |
| `rebuildBroadPhase(discardContacts?)` | بازسازی از روی ترنسفرم‌های فعلی. `loadSnapshot` آن را صدا می‌زند؛ پس از جابه‌جایی همزمان تعداد زیادی جسم هم مفید است. |

---

## Body

### `world.createBody(def)`

</div>

```ts
interface BodyDef {
  type?: BodyType;                        // پیش‌فرض Dynamic
  position?: { x: number; y: number };    // موقعیت جهانی مبدأ جسم
  angle?: number;                         // رادیان
  linearVelocity?: { x: number; y: number };
  angularVelocity?: number;               // rad/s
  linearDamping?: number;                 // 1/s
  angularDamping?: number;                // 1/s
  gravityScale?: number;                  // پیش‌فرض 1؛ مقدار 0 جسم را شناور می‌کند
  fixedRotation?: boolean;                // قفل چرخش (کاراکترهای پلتفرمر)
  allowSleep?: boolean;                   // پیش‌فرض true
  awake?: boolean;                        // پیش‌فرض true
  enabled?: boolean;                      // پیش‌فرض true
  bullet?: boolean;                       // فعال کردن برخورد پیوسته
  userData?: unknown;                     // موتور هرگز به آن دست نمی‌زند
}
```

<div dir="rtl">

### `BodyType`

| مقدار | حرکت با | جرم | برخورد با |
|---|---|---|---|
| `BodyType.Static` | مستقیماً توسط شما | بی‌نهایت | dynamic، kinematic |
| `BodyType.Kinematic` | سرعت خودش | بی‌نهایت | dynamic |
| `BodyType.Dynamic` | نیروها | متناهی | همه‌چیز |

### ترنسفرم

> API عمومی با **مبدأ** جسم کار می‌کند؛ حل‌کننده درونی **مرکز جرم** را
> انتگرال‌گیری می‌کند. `localCenter` / `worldCenter` پل بین این دو هستند.

| عضو | توضیح |
|---|---|
| `getPosition(): Vec2` | موقعیت جهانی مبدأ. |
| `getAngle(): Scalar` | چرخش در بازهٔ `(-π, π]`. |
| `setTransform(x, y, angle)` | جابه‌جایی آنی. سرعت حفظ می‌شود، جسم بیدار می‌شود. |
| `transform: Transform` | نمای فقط-خواندنی موقعیت + چرخش. |
| `worldCenter` / `localCenter: Vec2` | مرکز جرم. |
| `getWorldPoint(out, local)` / `getLocalPoint(out, world)` | تبدیل نقطه. |
| `getWorldVector(out, local)` / `getLocalVector(out, world)` | تبدیل جهت (بدون انتقال). |
| `getVelocityAtPoint(out, p): Vec2` | سرعت یک نقطهٔ جهانی روی این جسم. |

### سرعت و نیرو

| عضو | توضیح |
|---|---|
| `linearVelocity: Vec2`، `angularVelocity: Scalar` | مربوط به مرکز جرم. |
| `setLinearVelocity(vx, vy)` / `setAngularVelocity(w)` | setterهایی که جسم را هم بیدار می‌کنند. |

> **ورودی غیرمتناهی نادیده گرفته می‌شود.** همهٔ setterها و متدهای نیرو در
> `Body` مقادیر `NaN` و `Infinity` را بی‌صدا رد می‌کنند تا به حل‌کننده نرسند،
> جایی که از طریق تماس‌ها پخش می‌شد و جسم را بدون هیچ خطایی از کار می‌انداخت.
> `world.gravity` هم به همین دلیل یک بار در هر گام بررسی می‌شود.
| `applyForce(fx, fy, px?, py?, wake?)` | نیرو در یک نقطهٔ جهانی؛ تا گام بعد انباشته می‌شود. |
| `applyForceToCenter(fx, fy, wake?)` | نیرو بدون گشتاور. |
| `applyTorque(t, wake?)` | گشتاور خالص، نیوتن‌متر. |
| `applyLinearImpulse(ix, iy, px?, py?, wake?)` | تغییر آنی سرعت، نیوتن‌ثانیه. برای ضربه، پرش، انفجار. |
| `applyAngularImpulse(i, wake?)` | ضربهٔ زاویه‌ای. |

**نیرو در برابر ضربه:** نیرویی که هر فریم اعمال شود شتاب نرم تولید می‌کند؛ ضربه
سرعت را **همین حالا** عوض می‌کند.

### جرم

| عضو | توضیح |
|---|---|
| `mass`، `invMass`، `inertia`، `invInertia: Scalar` | فقط خواندنی. اینرسی حول مرکز جرم است. |
| `resetMassData()` | محاسبهٔ مجدد از روی فیکسچرها. هنگام تغییر فیکسچر خودکار است. |
| `setMassData(mass, inertia, cx?, cy?)` | بازنویسی دستی. |
| `clearMassOverride()` | بازگشت به مقادیر محاسبه‌شده. |
| `getKineticEnergy(): Scalar` | کل انرژی جنبشی، ژول. |

جسم dynamic بدون فیکسچر (یا با چگالی صفر) جرم `۱` می‌گیرد تا همچنان به نیروها
پاسخ دهد، به‌جای اینکه بی‌صدا غیرقابل حرکت شود.

### وضعیت

| عضو | توضیح |
|---|---|
| `awake: boolean`، `setAwake(b)` | اجسام خواب کاملاً نادیده گرفته می‌شوند. بیدار کردن، تایمر خواب را صفر می‌کند. |
| `enabled: boolean`، `setEnabled(b)` | جسم غیرفعال **منجمد** است: حل‌کننده ردش می‌کند و از فاز گسترده حذف می‌شود. موقعیت را **قبل** از فعال‌سازی مجدد تنظیم کنید. |
| `setType(t)` | تغییر نوع جسم؛ جرم و تماس‌ها بازسازی می‌شوند. |
| `setFixedRotation(b)` | قفل/آزاد کردن چرخش. |
| `bullet: boolean` | برخورد پیوسته برای این جسم. |
| `fixtures: Fixture[]` | فیکسچرهای متصل، به ترتیب ساخت. |
| `id: number` | اندیس متراکم پایدار. |
| `userData: unknown` | مال شما. |

---

## Fixture

یک شکل را با خواص مادی به یک جسم متصل می‌کند.

### `body.addFixture(def)`

</div>

```ts
interface FixtureDef {
  shape: Shape;             // الزامی؛ می‌تواند بین فیکسچرها مشترک باشد
  density?: number;         // kg/m²، پیش‌فرض 1
  friction?: number;        // پیش‌فرض 0.6؛ مقدار جفت = sqrt(a·b)
  restitution?: number;     // پیش‌فرض 0؛   مقدار جفت = max(a, b)
  isSensor?: boolean;       // پیش‌فرض false — فقط تشخیص هم‌پوشانی، بدون نیرو
  filter?: Partial<Filter>;
  tangentSpeed?: number;    // سرعت سطح نوار نقاله، m/s
  userData?: unknown;
}
```

<div dir="rtl">

| عضو | توضیح |
|---|---|
| `setFilter(f)` | تغییر فیلترینگ؛ جفت‌ها در گام بعد بازارزیابی می‌شوند. |
| `setDensity(d)` | سپس `body.resetMassData()` را صدا بزنید. |
| `testPoint(x, y): boolean` | تست دربرگیری دقیق. |
| `aabb: AABB` | AABB جهانی کش‌شده. |
| `body`، `shape`، `id` | فقط خواندنی. |

فراخوان `body.removeFixture(fixture)` آن را جدا و حذف می‌کند.

---

## شکل‌ها

شکل‌ها هندسهٔ تغییرناپذیر در فضای محلی هستند و آزادانه قابل اشتراک‌اند.

### Circle

</div>

```ts
Circle.of(radius: number, cx?: number, cy?: number): Circle
new Circle(radius: Scalar, center?: Vec2)
```

<div dir="rtl">

### Capsule

یک قطعه‌خط با شعاع — بهترین گزینه برای کنترلر کاراکتر، چون از پله بالا می‌رود و
روی دیوار می‌لغزد بدون اینکه به گوشه‌ها گیر کند.

</div>

```ts
Capsule.vertical(height: number, radius: number): Capsule    // ارتفاع کل
Capsule.horizontal(width: number, radius: number): Capsule
Capsule.of(x1, y1, x2, y2, radius): Capsule
```

<div dir="rtl">

### Polygon

محدب، خلاف عقربه‌های ساعت، حداکثر `MAX_POLYGON_VERTICES` (۸) رأس.

</div>

```ts
Polygon.box(halfWidth, halfHeight, radius?): Polygon
Polygon.offsetBox(hw, hh, cx, cy, angle?): Polygon
Polygon.regular(sides, radius, angleOffset?): Polygon
new Polygon(points: Vec2[], radius?: Scalar)
```

<div dir="rtl">

سازنده یک پوش محدب قطعی اجرا می‌کند، پس ورودی نامرتب، در جهت ساعت‌گرد یا کمی
مقعر همه شکل معتبری می‌سازند. نقاط تکراری و هم‌خط حذف می‌شوند. پارامتر اختیاری
`radius` گوشه‌ها را گرد می‌کند.

اگر کمتر از ۳ نقطهٔ غیرهم‌خط بماند، خطا می‌دهد.

### Segment

لبهٔ بدون ضخامت و **بدون جرم** — فقط هندسهٔ ثابت.

</div>

```ts
Segment.of(x1, y1, x2, y2): Segment
segment.setGhosts(prev: Vec2 | null, next: Vec2 | null)
```

<div dir="rtl">

### ChainShape

یک خط شکسته را به قطعه‌هایی با رأس‌های شبح تبدیل می‌کند تا جسمی که روی درزها
می‌لغزد گیر نکند.

</div>

```ts
ChainShape.fromPoints(points: Vec2[], loop?: boolean): Segment[]
```

<div dir="rtl">

> **جهت:** زنجیره یک‌طرفه است. سمت جامد، **سمت چپ جهت حرکت** است، پس خطی که از
> چپ به راست نوشته شود از بالا جامد است و یک حلقهٔ پادساعت‌گرد از داخل جامد است.
> برای برعکس کردن، ترتیب نقاط را معکوس کنید.

### رابط مشترک

</div>

```ts
interface Shape {
  readonly type: ShapeType;        // Circle | Capsule | Polygon | Segment
  readonly radius: Scalar;
  readonly vertexCount: number;
  computeAABB(out: AABB, xf: Transform): AABB;
  computeMass(out: MassData, density: Scalar): MassData;
  testPoint(xf: Transform, p: Vec2): boolean;
  rayCast(out: RayCastOutput, input: RayCastInput, xf: Transform): boolean;
  supportIndex(d: Vec2): number;
  getVertex(i: number): Vec2;
  clone(): Shape;
}
```

<div dir="rtl">

نوع `MassData` برابر `{ mass, center, inertia }` است، و اینرسی حول **مبدأ
محلی** محاسبه می‌شود (نه مرکز سطح).

---

## مفصل‌ها

همهٔ مفصل‌ها یک کلاس پایهٔ مشترک دارند:

</div>

```ts
interface JointDefBase {
  bodyA: Body;
  bodyB: Body;
  collideConnected?: boolean;   // پیش‌فرض false
  userData?: unknown;
}
```

<div dir="rtl">

| عضو | توضیح |
|---|---|
| `getAnchorA(out)` / `getAnchorB(out): Vec2` | لنگرگاه‌های جهانی. |
| `getReactionForce(out, invDt): Vec2` | نیرو روی جسم B، نیوتن. مقدار `world.invSubStep` را بدهید. |
| `getReactionTorque(invDt): Scalar` | گشتاور روی جسم B. مقدار `world.invSubStep` را بدهید. |
| `wake()` | بیدار کردن هر دو جسم. |
| `isActive(): boolean` | حداقل یک جسم بیدار و در حال شبیه‌سازی است. |

نیروی واکنش راهی است برای ساخت مفصل شکستنی: وقتی از یک آستانه گذشت مفصل را حذف
کنید.

> ضربه‌های مفصل **در هر گام فرعی** انباشته می‌شوند، پس ضریب تبدیل درست
> `world.invSubStep` است، نه `1 / timeStep`. با آن، یک بار آویزان دقیقاً وزن
> خودش را گزارش می‌کند، فارغ از مقدار `subSteps`.

### RevoluteJoint

لولا — دو جسم یک نقطه را مشترک دارند و آزادانه حولش می‌چرخند.

</div>

```ts
interface RevoluteJointDef extends JointDefBase, LimitDef, MotorDef, SpringDef {
  localAnchorA?: { x: number; y: number };
  localAnchorB?: { x: number; y: number };
  referenceAngle?: number;      // پیش‌فرض: زاویه در لحظهٔ ساخت
}
```

<div dir="rtl">

متدها: `getJointAngle()`، `getJointSpeed()`، `getMotorTorque(invDt)`،
`setLimits(lower, upper)`، `setMotorSpeed(w)`، `setMaxMotorTorque(t)`.

### PrismaticJoint

ریل کشویی — جسم B روی یک محور از A جابه‌جا می‌شود؛ حرکت عمود و چرخش نسبی قفل‌اند.

</div>

```ts
interface PrismaticJointDef extends JointDefBase, LimitDef, MotorDef, SpringDef {
  localAnchorA?, localAnchorB?: { x, y };
  localAxisA?: { x: number; y: number };   // خودکار نرمال می‌شود؛ پیش‌فرض (1,0)
  referenceAngle?: number;
}
```

<div dir="rtl">

متدها: `getJointTranslation()`، `getJointSpeed()`، `getMotorForce(invDt)`،
`setLimits(lo, hi)`، `setMotorSpeed(v)`.

### DistanceJoint

</div>

```ts
interface DistanceJointDef extends JointDefBase, SpringDef, LimitDef, MotorDef {
  localAnchorA?, localAnchorB?: { x, y };
  length?: number;        // پیش‌فرض: فاصلهٔ فعلی لنگرگاه‌ها
  minLength?: number;
  maxLength?: number;
  enableRigid?: boolean;  // پیش‌فرض true مگر enableSpring داده شود
}
```

<div dir="rtl">

سه حالت: **صلب** (طول ثابت)، **فنر** (`enableSpring` + `hertz` +
`dampingRatio`)، **طناب** (`enableLimit` با `minLength`/`maxLength`).

متدها: `getCurrentLength()`، `setLength(l)`، `setLengthRange(min, max)`.

### WeldJoint

هر سه درجهٔ آزادی را جوش می‌دهد. جوش کاملاً صلب را معمولاً بهتر است با یک جسم و
دو فیکسچر بسازید — دلیل استفاده از این مفصل حالت **نرم** است.

</div>

```ts
interface WeldJointDef extends JointDefBase {
  localAnchorA?, localAnchorB?: { x, y };
  referenceAngle?: number;
  linearHertz?: number;          // 0 = صلب
  linearDampingRatio?: number;
  angularHertz?: number;         // 0 = صلب
  angularDampingRatio?: number;
}
```

<div dir="rtl">

### MouseJoint

جسم B را با یک فنر نرم به سمت یک هدف جهانی می‌کشد. `bodyA` نادیده گرفته می‌شود
(معمولاً یک جسم ثابت زمین).

</div>

```ts
interface MouseJointDef extends JointDefBase {
  target?: { x: number; y: number };
  hertz?: number;          // پیش‌فرض 5
  dampingRatio?: number;   // پیش‌فرض 0.7
  maxForce?: number;       // پیش‌فرض 1000 نیوتن
}
```

<div dir="rtl">

متدها: `setTarget(x, y)`، `setTargetScalar(x, y)`.

> **شبکه:** هدف بخشی از جریان ورودی شماست و باید مثل هر ورودی دیگری ارسال و
> کوانتیزه شود.

### MotorJoint

جسم B را با نیرو و گشتاور محدود به یک آفست و زاویهٔ هدف نسبت به A می‌رساند. برای
کاراکترهای kinematic و سکوهای متحرکی که باید همچنان به برخورد احترام بگذارند —
برخلاف جابه‌جایی آنی، جسم به‌طور طبیعی پشت مانع می‌ایستد.

</div>

```ts
interface MotorJointDef extends JointDefBase {
  linearOffset?: { x: number; y: number };
  angularOffset?: number;
  maxForce?: number;           // پیش‌فرض 1000
  maxTorque?: number;          // پیش‌فرض 1000
  correctionFactor?: number;   // 0..1، پیش‌فرض 0.3
}
```

<div dir="rtl">

متدها: `setLinearOffset(x, y)`، `setAngularOffset(a)`.

### گروه‌های تنظیم مشترک

</div>

```ts
interface SpringDef { enableSpring?: boolean; hertz?: number; dampingRatio?: number }
interface MotorDef  { enableMotor?: boolean; motorSpeed?: number; maxMotorForce?: number }
interface LimitDef  { enableLimit?: boolean; lowerLimit?: number; upperLimit?: number }
```

<div dir="rtl">

مقدار `dampingRatio`: کمتر از ۱ کم‌میرا (جهنده)، برابر ۱ میرایی بحرانی، بیشتر از
۱ فوق‌میرا.

---

## تماس و رویداد

</div>

```ts
world.setListener({
  beginContact(e: ContactEvent) {},
  endContact(e: ContactEvent) {},
  beginSensor(e: ContactEvent) {},
  endSensor(e: ContactEvent) {},
  preSolve(e: ContactEvent) {},   // قبل از حل — می‌تواند تماس را غیرفعال کند
  postSolve(e: ImpactEvent) {},   // بعد از حل — حاوی ضربه‌ها
});
```

```ts
interface ContactEvent { fixtureA: Fixture; fixtureB: Fixture; contact: Contact }

interface ImpactEvent extends ContactEvent {
  maxNormalImpulse: Scalar;   // بزرگ‌ترین ضربه روی manifold، نیوتن‌ثانیه
  approachSpeed: Scalar;      // سرعت نزدیک شدن قبل از برخورد، m/s (منفی)
}
```

<div dir="rtl">

> ⚠️ **آبجکت‌های رویداد بازاستفاده می‌شوند.** رویداد `preSolve` برای هر تماس در
> هر گام یک بار اجرا می‌شود، پس ساختن یک آبجکت تازه هر بار هزینهٔ محسوسی در
> جمع‌آوری زباله داشت. آبجکتی که به callback داده می‌شود در فراخوانی بعدی
> بازنویسی می‌شود — فیلدهای لازم را کپی کنید، نه خود رویداد را.
>
> </div>
>
> ```ts
> postSolve(e) {
>   if (Scalar.toFloat(e.maxNormalImpulse) > 5) {
>     hits.push({ id: e.fixtureA.body.id });   // ✓ کپی
>   }
> }
> ```

<div dir="rtl">

### Contact

| عضو | توضیح |
|---|---|
| `isTouching: boolean` | شکل‌ها واقعاً هم‌پوشانی دارند (نه فقط AABBها). |
| `isSensor: boolean` | یکی از فیکسچرها سنسور است. |
| `setEnabled(b)` / `isEnabled` | غیرفعال کردن فقط برای گام جاری — ترفند سکوی یک‌طرفه. |
| `manifold: Manifold` | نقاط تماس و نرمال (نرمال از **A به B**). |
| `friction`، `restitution`، `tangentSpeed: Scalar` | مقادیر ترکیبی جفت. |
| `getTotalNormalImpulse(): Scalar` | جمع روی نقاط manifold. |

### Manifold

</div>

```ts
class Manifold {
  points: [ManifoldPoint, ManifoldPoint];
  normal: Vec2;        // از A به B
  pointCount: number;  // ۰، ۱ یا ۲
}

class ManifoldPoint {
  point: Vec2;               // فضای جهانی
  separation: Scalar;        // منفی = هم‌پوشانی
  normalImpulse: Scalar;     // انباشته، برای warm starting باقی می‌ماند
  tangentImpulse: Scalar;
  maxNormalImpulse: Scalar;
  relativeVelocity: Scalar;  // قبل از حل ثبت شده
  id: ContactID;             // شناسهٔ ویژگی — نقاط را بین گام‌ها تطبیق می‌دهد
  persisted: boolean;
}
```

<div dir="rtl">

در دو بعد، دو نقطه برای نمایش هم «رأس روی وجه» و هم «دو وجه تخت در تماس» کافی
است.

---

## فیلترینگ

</div>

```ts
interface Filter {
  category: number;   // بیت‌فیلد: این فیکسچر جزو کدام دسته‌هاست
  mask: number;       // بیت‌فیلد: با کدام دسته‌ها برخورد می‌کند
  group: number;      // بازنویسی؛ پایین را ببینید
}
```

<div dir="rtl">

به این ترتیب ارزیابی می‌شود:

۱. **گروه** — دو فیکسچر با گروه یکسان و غیرصفر همیشه (مثبت) یا هرگز (منفی)
   برخورد می‌کنند و ماسک‌ها را بی‌اثر می‌کنند.
۲. **دسته/ماسک** — در غیر این صورت هر دو جهت باید موافق باشند:
   `A.mask & B.category` و `B.mask & A.category` هر دو باید غیرصفر باشند.

توابع کمکی: `makeFilter(partial?)`، `shouldCollide(a, b)`، `DEFAULT_FILTER`.

---

## ریاضیات

### Vec2

جفت `{x, y}` قابل تغییر. عملگرها دو شکل دارند: **تخصیص‌دهنده** (`Vec2.add`) و
**مقصددار** (`Vec2.addTo(out, …)`) — حل‌کننده فقط از دومی استفاده می‌کند و به
همین دلیل یک گام در حالت پایدار هیچ تخصیص حافظه‌ای ندارد.

</div>

```ts
Vec2.of(x: number, y: number): Vec2      // از اعداد ممیز شناور ساده
Vec2.zero(): Vec2
v.toFloats(): { x: number; y: number }   // بازگشت به اعداد ساده
```

<div dir="rtl">

| متد استاتیک | معنی |
|---|---|
| `add`، `sub`، `scale`، `neg` | تخصیص‌دهنده. |
| `addTo`، `subTo`، `scaleTo`، `negTo`، `lerpTo`، `minTo`، `maxTo` | نوشتن در `out`. |
| `addScaledTo(out, a, b, s)` | `out = a + b·s` — اسب بارکش حل‌کننده. |
| `combineTo(out, a, sa, b, sb)` | `out = a·sa + b·sb`. |
| `dot(a, b)`، `cross(a, b)` | ضرب‌ها (`cross` مؤلفهٔ z اسکالر است). |
| `crossVS`، `crossSV`، `perpTo`، `rperpTo` | چرخش‌های ±۹۰ درجه. |
| `distance(a, b)`، `distanceSq(a, b)` | سنجه‌ها. |
| `normalizeTo(out, v): Scalar` | نرمال‌سازی؛ طول اولیه را برمی‌گرداند. |
| `equals(a, b)` | برابری دقیق مؤلفه‌ها. |

متدهای نمونه: `set`، `setZero`، `copyFrom`، `clone`، `add`، `sub`، `addScaled`،
`scale`، `neg`، `length`، `lengthSq`، `normalize`، `truncate`، `isZero`،
`isValid`.

متد `normalize()` طول **قبلی** را برمی‌گرداند و بردار صفر را دست‌نخورده رها
می‌کند، پس هرگز لازم نیست مراقب NaN باشید.

### Rot

چرخشی که به‌صورت `(sin θ, cos θ)` ذخیره می‌شود، تا حل‌کننده در حلقه‌های داخلی‌اش
هرگز تابع مثلثاتی صدا نزند.

</div>

```ts
new Rot(angle?: Scalar)
Rot.of(angle: number): Rot
```

<div dir="rtl">

متدها: `setAngle`، `setSinCos`، `setIdentity`، `getAngle`، `getXAxis`،
`getYAxis`، `normalize`، `integrate(dAngle)`، `copyFrom`، `clone`.
استاتیک‌ها: `Rot.mulTo`، `mulTTo`، `rotate`، `rotateT`، `relativeAngle`،
`nlerpTo`.

متد `integrate` با نگاشت نمایی زاویهٔ کوچک جلو می‌رود و دوباره نرمال می‌کند —
ارزان‌تر از محاسبهٔ مجدد `sinCos` و در اجراهای طولانی پایدار.

### Transform

انتقال `p` + چرخش `q` که فضای محلی را به فضای جهانی می‌برد.

</div>

```ts
new Transform(p?: Vec2, q?: Rot)
Transform.apply(out, xf, v)     // نقطهٔ محلی → نقطهٔ جهانی
Transform.applyT(out, xf, v)    // نقطهٔ جهانی → نقطهٔ محلی
Transform.mulTo(out, a, b)      // ترکیب
Transform.mulTTo(out, a, b)     // b در دستگاه a
```

<div dir="rtl">

### Mat22 / Mat33

حل‌کنندهٔ ۲×۲ و ۳×۳ متقارن برای بلوک‌های قید جفت‌شده.
متدها: `set`، `det`، `solve(out, b)`، `invertTo(out)`، `Mat22.apply(out, m, v)`؛
و `Mat33.solve22`، `solve33`. سیستم‌های تکین به‌جای NaN مقدار صفر می‌دهند.

### مثلثات قطعی

</div>

```ts
sin(a), cos(a), tan(a)
sinCos(a, out)        // هر دو با هم — تقریباً ۲ برابر سریع‌تر از دو فراخوانی
atan(t), atan2(y, x), asin(v), acos(v)
normalizeAngle(a)     // بردن به بازهٔ (-π, π]
```

<div dir="rtl">

پیاده‌سازی‌های چندجمله‌ای که روی هر پلتفرم نتیجهٔ یکسان می‌دهند. برای دقت،
[DETERMINISM.md](DETERMINISM.md#۲۱-توابع-مقدماتی-transcendental) را ببینید.

مقدار `atan2(0, 0)` برابر `0` تعریف شده؛ توابع `asin`/`acos` ورودی خارج از بازه
را می‌برند داخل بازه.

### Rng

مولد seed‌دار از خانوادهٔ PCG: پیشروی حالت با LCG به‌همراه finalizer از نوع
murmur3. توزیع یکنواخت با آزمون کای‌دو، و کمتر از ۰٫۴٪ اریبی روی هر بیت.

</div>

```ts
const rng = new Rng(seed?, stream?);
rng.next(): number             // uint32 خام
rng.float(): number            // [0, 1)
rng.scalar(lo, hi): Scalar     // اسکالر بک‌اند در یک بازه
rng.int(lo, hi): number        // شامل دو سر، بدون اریبی باقیمانده
rng.bool(p?): boolean
rng.shuffle(array): T[]        // فیشر–ییتس، قطعی
rng.getState(): [number, number]
rng.setState(s, i): void
rng.seed(n): void
```

<div dir="rtl">

### Scalar

</div>

```ts
import { Scalar as S } from 'pulse2d';

S.fromFloat(x) / S.toFloat(x)        // تبدیل — همیشه از این‌ها استفاده کنید
S.fromInt(i) / S.toInt(x)
S.mul, S.div, S.inv, S.sqrt, S.mulAdd(a, b, c)   // a*b + c
S.half, S.sq, S.abs, S.min, S.max, S.clamp, S.sign, S.lerp
S.ZERO, S.ONE, S.TWO, S.HALF, S.PI, S.TWO_PI, S.HALF_PI, S.EPSILON
S.BACKEND       // 'f64' | 'q16.16'
S.IS_FIXED      // boolean
```

<div dir="rtl">

عملگرهای `+`، `-`، منهای یکانی و مقایسه‌ها در هر دو بک‌اند بومی کار می‌کنند و
برای سرعت مستقیم نوشته شده‌اند؛ فقط `mul`/`div`/`sqrt` به ماژول نیاز دارند.

---

## شبکه

راهنمای یکپارچه‌سازی: [NETWORKING.md](NETWORKING.md)

### Snapshot

</div>

```ts
saveSnapshot(world, reuse?): Snapshot
loadSnapshot(world, snap): void          // در عدم تطابق پروتکل/بک‌اند خطا می‌دهد
cloneSnapshot(snap): Snapshot            // کپی عمیق، برای بافر تاریخچه
snapshotBytes(snap): number

interface Snapshot { tick: number; data: Float64Array; meta: Int32Array }
```

<div dir="rtl">

ترنسفرم و سرعت اجسام، وضعیت خواب، ضربه‌های تماس، ضربه‌های مفصل و حالت RNG را
ثبت می‌کند. متد `loadSnapshot` فاز گسترده را بازمی‌سازد و تماس‌ها را دوباره کشف
می‌کند، پس دنیای بازیابی‌شده تابعی خالص از snapshot است.

دنیا باید **همان مجموعهٔ اجسام** (با همان شناسه‌ها) را داشته باشد که موقع گرفتن
snapshot داشت.

### Checksum

</div>

```ts
checksumWorld(world, positionsOnly?): number   // FNV-1a روی بیت‌های خام
checksumSnapshot(snap): number
new Hasher().int(n).float(x).scalar(s).digest() / .hex()

const log = new ChecksumLog(capacity?);
log.record(tick, sum) / log.recordWorld(world) / log.get(tick)
log.findDivergence(remote: Map<number, number>): number   // اولین tick بد، یا -1
log.toMap(): Map<number, number>
```

<div dir="rtl">

### RollbackManager

</div>

```ts
const rb = new RollbackManager(world, {
  maxRollbackFrames: 16,
  applyInputs: (tick, inputs: Map<number, I>) => void,    // باید خالص باشد
  predictInput?: (tick, playerId, last) => I | undefined, // پیش‌فرض: تکرار آخرین
  inputsEqual?: (a, b) => boolean,                        // پیش‌فرض: Object.is
  enableChecksums?: boolean,
  onRollback?: (fromTick, toTick, frames) => void,
});
```

<div dir="rtl">

| عضو | توضیح |
|---|---|
| `addPlayer(id)` | ثبت بازیکن تا اولین ورودی‌اش قابل پیش‌بینی و اصلاح باشد. |
| `addLocalInput(id, input)` | صف کردن برای `advance()` بعدی. |
| `addRemoteInput(tick, id, input)` | تحویل ورودی معتبر؛ در پیش‌بینی غلط خودکار rollback می‌کند. |
| `advance(): number` | snapshot، اعمال ورودی، یک گام. |
| `rollbackTo(tick): boolean` | عقب‌گرد و بازپخش دستی. اگر خیلی قدیمی باشد `false`. |
| `tick`، `oldestTick`، `historyLength`، `historyBytes` | بازرسی. |
| `rollbackCount`، `resimulatedTicks` | آمار تشخیصی. |
| `checksums: ChecksumLog` | وقتی `enableChecksums` فعال باشد پر می‌شود. |
| `reset()` | پاک کردن کل تاریخچه (پس از همگام‌سازی سخت). |

---

## رندر دیباگ

</div>

```ts
const draw = new DebugDraw(ctx: CanvasRenderingContext2D, {
  pixelsPerMeter?: number;   // پیش‌فرض 32
  cameraX?, cameraY?: number;
  lineWidth?: number;        // به پیکسل، مستقل از بزرگ‌نمایی ثابت می‌ماند
  flags?: Partial<DebugDrawFlags>;
  colors?: Partial<DebugDrawColors>;
});
```

<div dir="rtl">

| عضو | توضیح |
|---|---|
| `begin(clear?)` / `end()` | نصب/بازگرداندن تبدیل جهان→صفحه (**محور y+ رو به بالا**). |
| `drawWorld(world)` | رسم همهٔ لایه‌های فعال. |
| `drawBody(body)`، `drawShape(shape, xf, color, dashed?)` | اجزای جداگانه. |
| `drawContacts(world)`، `drawJoint(joint)`، `drawTree(world)` | لایه‌های رویی. |
| `strokeAABB`، `fillDot`، `drawCross`، `drawArrow`، `drawStats` | ابتدایی‌ها. |
| `screenToWorld(px, py)` / `worldToScreen(x, y)` | تبدیل مختصات — برای انتخاب با ماوس. |

پرچم‌ها: `shapes`، `fill`، `joints`، `contacts`، `contactNormals`،
`contactImpulses`، `aabbs`، `centerOfMass`، `sleepState`، `velocities`،
`stats`.

---

## درون فاز برخورد

برای ابزارهای سفارشی مفید است؛ برای کد معمول بازی لازم نیست.

### AABB

متدها: `set`، `copyFrom`، `clone`، `setEmpty`، `getCenter`، `getExtents`،
`perimeter`، `area`، `expand`، `addPoint`، `contains`، `containsPoint`،
`isValid`، `rayCast(p1, d, maxFraction)`؛ استاتیک‌ها `AABB.combineTo`،
`combinedPerimeter`، `overlaps`.

### فاز باریک

</div>

```ts
collide(manifold, shapeA, xfA, shapeB, xfB): void
```

<div dir="rtl">

بر اساس نوع شکل توزیع می‌کند و همیشه نرمالی تولید می‌کند که از **A به B** اشاره
دارد. جداگانه هم صادر شده‌اند: `collideCircles`، `collidePolygonCircle`،
`collidePolygons`.

### فاصله و جاروب شکل

</div>

```ts
shapeDistance(out, { proxyA, proxyB, xfA, xfB, useRadii }): DistanceOutput
shapeCast(out, { proxyA, proxyB, xfA, xfB, translationB, maxFraction }): boolean
makeProxy(shape), makeDistanceOutput(), makeShapeCastOutput()
```

<div dir="rtl">

فاصلهٔ GJK بین شکل‌های محدب، و یک جاروب conservative advancement روی آن. هر دو
سقف تکرار ثابت دارند، پس هزینه و نتیجه کران‌دار و بازتولیدپذیرند.

### فاز گسترده

</div>

```ts
new DynamicTree(capacity?)
  createProxy(aabb, userData) / destroyProxy(id) / moveProxy(id, aabb, margin, displacement)
  query(aabb, cb) / queryPoint(p, cb) / rayCast(p1, p2, maxFraction, cb)
  getAABB(out, id) / getUserData(id) / getHeight() / getQuality() / validate()
  proxyCount / nodeCount

new BroadPhase(capacity?)
  createProxy / destroyProxy / moveProxy / touchProxy
  updatePairs(cb) / query / queryPoint / rayCast / rebuild / clear
```

<div dir="rtl">

یک BVH با درج مبتنی بر اکتشاف سطح (SAH) و چرخش‌های AVL، ذخیره‌شده در آرایه‌های
تایپ‌دار تخت. متد `validate()` اگر درخت سالم باشد `null` برمی‌گرداند، وگرنه شرح
مشکل — در تست‌ها به‌کار می‌آید.

---

## تنظیمات

</div>

```ts
import { Settings } from 'pulse2d';
```

<div dir="rtl">

ثابت‌های زمان-کامپایل، با واحد متر/کیلوگرم/ثانیه. همهٔ همتاها باید بر سر آن‌ها
توافق داشته باشند؛ اگر یکی را عوض کردید `PROTOCOL_VERSION` را بالا ببرید.

| ثابت | پیش‌فرض | معنی |
|---|---|---|
| `LINEAR_SLOP` | ۰٫۰۰۵ | هم‌پوشانی مجاز — جلوی لرزش تماس را می‌گیرد. |
| `ANGULAR_SLOP` | ۲ درجه | معادل زاویه‌ای. |
| `SPECULATIVE_DISTANCE` | ۴ × slop | بازه‌ای که در آن تماس احتمالی ساخته می‌شود. |
| `AABB_MARGIN` | ۰٫۱ | حاشیهٔ فاز گسترده. |
| `MAX_TRANSLATION` | ۴ | سقف جابه‌جایی در هر گام. |
| `MAX_ROTATION` | ۰٫۵π | سقف چرخش در هر گام. |
| `MAX_BIAS_VELOCITY` | −۴ | سقف سرعت بیرون‌راندن هم‌پوشانی. |
| `RESTITUTION_THRESHOLD` | ۱ | زیر این سرعت نزدیک شدن، جهشی در کار نیست. |
| `SLEEP_LINEAR_TOLERANCE` | ۰٫۰۱ | آستانهٔ سرعت خواب. |
| `SLEEP_ANGULAR_TOLERANCE` | ۲ درجه بر ثانیه | آستانهٔ چرخش خواب. |
| `TIME_TO_SLEEP` | ۰٫۵ | ثانیه زیر آستانه پیش از خوابیدن. |
| `CONTACT_HERTZ` | ۳۰ | سفتی تماس. |
| `CONTACT_DAMPING_RATIO` | ۱۰ | به‌شدت فوق‌میرا، جهش را سرکوب می‌کند. |
| `JOINT_HERTZ` | ۶۰ | سفتی پیش‌فرض مفصل. |
| `JOINT_DAMPING_RATIO` | ۲ | میرایی پیش‌فرض مفصل. |
| `PROTOCOL_VERSION` | ۲ | در هدر snapshot درج می‌شود. |

همچنین صادر می‌شود: `VERSION`، رشتهٔ نسخهٔ کتابخانه.

</div>
