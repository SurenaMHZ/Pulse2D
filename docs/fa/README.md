<div align="center" dir="rtl">

# Pulse2D

**موتور فیزیک دوبعدی سریع، ماژولار و کاملاً قطعی برای بازی‌های بلادرنگ**

تایپ‌اسکریپت · بدون وابستگی · ۳۳ کیلوبایت فشرده · نتیجهٔ یکسان روی هر دستگاه

[![CI](https://github.com/SurenaMHZ/pulse2d/actions/workflows/ci.yml/badge.svg)](https://github.com/SurenaMHZ/pulse2d/actions/workflows/ci.yml)
[![npm](https://img.shields.io/npm/v/pulse2d.svg)](https://www.npmjs.com/package/pulse2d)
[![licence](https://img.shields.io/badge/licence-MIT-blue)](../../LICENSE)

[شروع سریع](#شروع-سریع) · [مستندات](#مستندات) · [قطعیت](DETERMINISM.md) · [مرجع API](API.md) · [کارایی](#کارایی)

[English documentation](../../README.md)

</div>

---

<div dir="rtl">

## چرا Pulse2D

بیشتر موتورهای فیزیک جاوااسکریپت روی هر دستگاه نتیجهٔ **تقریباً** یکسان می‌دهند.
برای بازی تک‌نفره مشکلی نیست، ولی برای مولتی‌پلیر lockstep فاجعه است: یک بیت
اختلاف در آخرین رقم اعشار، ظرف چند ثانیه به دو دنیای کاملاً متفاوت تبدیل می‌شود.

Pulse2D از ابتدا برعکس ساخته شده — **اول قطعیت**:

- **نتیجهٔ بیت‌به‌بیت یکسان.** فقط از پنج عملیاتی استفاده می‌شود که استاندارد
  IEEE-754 گرد کردن دقیقشان را **الزامی** کرده است (`+ - * / sqrt`). توابع
  `Math.sin`، `Math.atan2` و مشابه در کل موتور ممنوع‌اند و با پیاده‌سازی چندجمله‌ای
  خودمان جایگزین شده‌اند، چون نتیجه‌شان بین مرورگرها فرق می‌کند.
- **بدون حالت پنهان.** ترتیب حل قیدها تابعی از **وضعیت** دنیاست، نه از تاریخچهٔ
  کشف آن‌ها — پس کلاینتی که با rollback و بازپخش به یک وضعیت می‌رسد، دقیقاً همان
  چیزی را حساب می‌کند که کلاینتی که مستقیم به آن رسیده.
- **rollback داخلی.** snapshot، checksum و درایور rollback از نوع GGPO داخل
  کتابخانه هستند، نه یک افزودنی بعدی.
- **دو بک‌اند عددی.** به‌طور پیش‌فرض Float64؛ و یک نسخهٔ Q16.16 ممیزثابت برای
  وقتی که می‌خواهید ممیز شناور را کاملاً کنار بگذارید.

هر چیز دیگری هم که انتظار دارید هست: دایره، کپسول، چندضلعی محدب، زنجیره، شش نوع
مفصل، سنسور، ray cast، خوابیدن اجسام، برخورد پیوسته و یک رندرر دیباگ Canvas.

---

## نصب

</div>

```bash
npm install pulse2d
```

```ts
import { World, BodyType, Polygon, Circle } from 'pulse2d';
```

<div dir="rtl">

نسخهٔ ممیزثابت نقطهٔ ورود جداگانه دارد:

</div>

```ts
import { World } from 'pulse2d/fixed';
```

<div dir="rtl">

بدون مرحلهٔ build؟ از باندل UMD استفاده کنید:

</div>

```html
<script src="./dist/pulse2d.umd.js"></script>
<script>const world = new Pulse2D.World({ gravity: { x: 0, y: -10 } });</script>
```

---

<div dir="rtl">

## شروع سریع

</div>

```ts
import { World, BodyType, Polygon, Circle } from 'pulse2d';

// ۱. یک دنیا. گام زمانی موقع ساخت ثابت می‌شود — بخش «گام زمانی ثابت» را ببینید.
const world = new World({ gravity: { x: 0, y: -10 } });

// ۲. زمین ثابت که سطح بالایش روی y = 0 است.
const ground = world.createBody({ type: BodyType.Static, position: { x: 0, y: -1 } });
ground.addFixture({ shape: Polygon.box(50, 1), friction: 0.6 });

// ۳. یک توپ جهنده.
const ball = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 8 } });
ball.addFixture({ shape: Circle.of(0.5), density: 1, restitution: 0.6 });

// ۴. گام برداشتن.
for (let i = 0; i < 120; i++) world.step();

console.log(ball.getPosition().toFloats()); // { x: 0, y: ~1.4 }
```

<div dir="rtl">

### واحدها

Pulse2D با **متر، کیلوگرم و ثانیه** کار می‌کند و برای اجسامی در حدود
`۰٫۱ تا ۱۰ متر` تنظیم شده است. اگر بازی شما با پیکسل فکر می‌کند، موقع ورود بر یک
عدد ثابت (معمولاً ۳۰ تا ۱۰۰) تقسیم کنید و موقع خروج ضرب کنید. اگر یک جعبهٔ
۱۰۰۰ پیکسلی را مستقیم شبیه‌سازی کنید، حرکتش شبیه فروریختن یک ساختمان می‌شود —
چون دقیقاً همین را خواسته‌اید.

---

## مفاهیم اصلی

### جسم، فیکسچر و شکل

سه لایه، هرکدام با یک وظیفه:

| لایه | مالکِ | قابل اشتراک؟ |
|---|---|---|
| **Shape** | هندسه در فضای محلی | بله — یک `Polygon` را بین هزار جعبه به اشتراک بگذارید |
| **Fixture** | چگالی، اصطکاک، ارتجاع، فیلترینگ، پرچم سنسور | نه — یکی برای هر اتصال به جسم |
| **Body** | موقعیت، سرعت، جرم | نه |

</div>

```ts
const crateShape = Polygon.box(0.5, 0.5);      // هندسهٔ مشترک

for (let i = 0; i < 1000; i++) {
  const body = world.createBody({ type: BodyType.Dynamic, position: { x: i, y: 5 } });
  body.addFixture({ shape: crateShape, density: 1, friction: 0.5 });
}
```

<div dir="rtl">

برای ساختن اجسام **مرکب** (مقعر)، چند فیکسچر به یک جسم وصل کنید:

</div>

```ts
const table = world.createBody({ type: BodyType.Dynamic, position: { x: 0, y: 2 } });
table.addFixture({ shape: Polygon.offsetBox(1.0, 0.1,  0.0, 0.5) }); // رویه
table.addFixture({ shape: Polygon.offsetBox(0.1, 0.5, -0.8, 0.0) }); // پایهٔ چپ
table.addFixture({ shape: Polygon.offsetBox(0.1, 0.5,  0.8, 0.0) }); // پایهٔ راست
```

<div dir="rtl">

### انواع جسم

| نوع | حرکت با | جرم | برخورد با |
|---|---|---|---|
| `Static` | مستقیماً توسط شما | بی‌نهایت | زمین، دیوار — رایگان، هرگز شبیه‌سازی نمی‌شود |
| `Kinematic` | سرعت خودش | بی‌نهایت | سکوی متحرک، آسانسور — هل می‌دهد، هل داده نمی‌شود |
| `Dynamic` | نیروها | متناهی | هر چیز دیگر |

### شکل‌ها

</div>

```ts
Circle.of(radius, cx?, cy?)                    // ارزان‌ترین شکل
Capsule.vertical(height, radius)               // بهترین گزینه برای کاراکتر
Capsule.horizontal(width, radius)
Polygon.box(halfWidth, halfHeight)             // محدب، حداکثر ۸ رأس
Polygon.offsetBox(hw, hh, cx, cy, angle?)
Polygon.regular(sides, radius)
new Polygon(points)                            // پوش محدب خودکار گرفته می‌شود
Segment.of(x1, y1, x2, y2)                     // بدون جرم، فقط هندسهٔ ثابت
ChainShape.fromPoints(points, loop?)           // زمین، با رأس‌های شبح
```

<div dir="rtl">

سازندهٔ `new Polygon(points)` یک الگوریتم پوش محدب قطعی اجرا می‌کند، پس حتی اگر
نقاط را نامرتب یا کمی مقعر بدهید، شکل معتبری تحویل می‌گیرید.

برای زمین، به‌جای قطعه‌های جدا از `ChainShape` استفاده کنید — رأس‌های شبح را
وصل می‌کند تا جعبه‌ای که روی درزها می‌لغزد گیر نکند.

</div>

```ts
const ground = world.createBody({ type: BodyType.Static });
const contour = [Vec2.of(-50, 0), Vec2.of(0, -2), Vec2.of(50, 0)];
for (const seg of ChainShape.fromPoints(contour)) {
  ground.addFixture({ shape: seg, friction: 0.8 });
}
```

<div dir="rtl">

> **جهت چرخش مهم است.** زنجیره یک‌طرفه است: سمت جامد، **سمت چپ جهت حرکت** است.
> پس خطی که از چپ به راست نوشته شود، از بالا جامد است. برای برعکس کردن، ترتیب
> نقاط را معکوس کنید.

### خواص مواد

</div>

```ts
body.addFixture({
  shape: Circle.of(0.5),
  density: 1,        // kg/m² — جرم و ممان اینرسی را تعیین می‌کند
  friction: 0.6,     // ۰ = یخ، ۱+ = لاستیک؛ مقدار جفت = sqrt(a·b)
  restitution: 0.4,  // ۰ = گِل، ۱ = کاملاً کشسان؛ مقدار جفت = max(a, b)
  isSensor: false,   // true = فقط تشخیص هم‌پوشانی، بدون نیرو
  tangentSpeed: 0,   // مقدار غیرصفر سطح را به نوار نقاله تبدیل می‌کند
});
```

<div dir="rtl">

اصطکاک از **میانگین هندسی** استفاده می‌کند تا یک سطح خیلی لغزنده بر جفت غالب
شود، که با شهود جور در می‌آید. ارتجاع از **بیشینه** استفاده می‌کند تا یک توپ
جهنده روی زمینِ بی‌جان هم بجهد.

---

## گام زمانی ثابت

متد `world.step()` هیچ delta time نمی‌گیرد. این عمدی است: `dt` متغیر نتیجه را
به نرخ فریم وابسته می‌کند و هم قطعیت و هم پایداری را از بین می‌برد.

برای اجرا از داخل حلقهٔ رندر متغیر، از `accumulate` استفاده کنید:

</div>

```ts
let last = performance.now();

function frame(now) {
  const dt = (now - last) / 1000;
  last = now;

  // بین ۰ تا maxSteps گام کامل اجرا می‌کند و کسر باقیمانده را برمی‌گرداند.
  const alpha = world.accumulate(dt, 5);

  render(alpha); // برای تصویر نرم، بین دو وضعیت آخر درون‌یابی کنید
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);
```

<div dir="rtl">

سقف `maxSteps` جلوی مارپیچ مرگ را می‌گیرد: فریم کند باعث گام‌های اضافه می‌شود و
آن هم فریم بعدی را کندتر می‌کند.

---

## رویدادها

</div>

```ts
world.setListener({
  beginContact({ fixtureA, fixtureB }) { /* شروع تماس */ },
  endContact({ fixtureA, fixtureB })   { /* پایان تماس */ },
  beginSensor({ fixtureA, fixtureB })  { /* ورود به ناحیهٔ تریگر */ },
  endSensor({ fixtureA, fixtureB })    { /* خروج از آن */ },

  // بعد از ساخت manifold و قبل از حل صدا زده می‌شود.
  preSolve({ contact }) {
    // سکوی یک‌طرفه: این تماس را برای یک گام نادیده بگیر.
    if (playerIsJumpingUp) contact.setEnabled(false);
  },

  // بعد از حل، با ضربه‌هایی که واقعاً اعمال شده‌اند.
  postSolve({ maxNormalImpulse, approachSpeed, fixtureA, fixtureB }) {
    if (maxNormalImpulse > 5) playCrashSound(maxNormalImpulse);
  },
});
```

<div dir="rtl">

با `postSolve` می‌توانید **شدت** برخورد را تشخیص دهید — یک تماس سبک و یک تصادف
شاخ‌به‌شاخ هر دو `beginContact` را صدا می‌زنند، ولی فقط یکی ضربهٔ بزرگی دارد.

> ⚠️ **آبجکت‌های رویداد بازاستفاده می‌شوند.** رکوردی که به callback داده می‌شود
> فقط در طول همان فراخوانی معتبر است؛ هر چیزی را که لازم دارید کپی کنید و خود
> آبجکت را ذخیره نکنید.

---

## فیلتر کردن برخورد

دو سازوکار مستقل، به همین ترتیب ارزیابی می‌شوند:

</div>

```ts
// ۱. گروه‌ها: دو فیکسچر با گروه یکسان و غیرصفر همیشه (مثبت) یا هرگز (منفی) برخورد می‌کنند.
//    عالی برای «همهٔ اجزای این ragdoll همدیگر را نادیده بگیرند».
body.addFixture({ shape, filter: { group: -1 } });

// ۲. بیت‌فیلدهای دسته/ماسک: هر دو جهت باید موافق باشند.
const PLAYER = 0x0001, ENEMY = 0x0002, PICKUP = 0x0004;

body.addFixture({
  shape,
  filter: { category: PLAYER, mask: ENEMY | PICKUP }, // با دشمن و آیتم برخورد می‌کند
});
```

---

<div dir="rtl">

## مفصل‌ها

| مفصل | چه چیزی را مقید می‌کند | کاربرد رایج |
|---|---|---|
| `RevoluteJoint`  | نقطهٔ مشترک، چرخش آزاد | لولا، چرخ، آرنج ragdoll |
| `PrismaticJoint` | حرکت روی یک محور | آسانسور، پیستون، درِ کشویی |
| `DistanceJoint`  | فاصلهٔ بین دو لنگرگاه | طناب، فنر، سیستم تعلیق |
| `WeldJoint`      | هر سه درجهٔ آزادی | سازه‌های شکستنی (در حالت نرم) |
| `MouseJoint`     | کشیدن به سمت یک هدف | درگ، پرتو کششی، آهنربا |
| `MotorJoint`     | رساندن به یک آفست هدف | سکوی متحرک آگاه به برخورد |

</div>

```ts
// لولا در یک نقطهٔ جهانی — لنگرگاه‌های محلی خودکار محاسبه می‌شوند.
const hinge = world.createRevoluteJointAt(chassis, wheel, wheelX, wheelY, {
  enableMotor: true,
  motorSpeed: -20,        // rad/s
  maxMotorForce: 500,     // بیشینهٔ گشتاوری که موتور می‌تواند اعمال کند
});

// طنابی که شل آویزان است ولی هرگز بیش از ۴ متر کشیده نمی‌شود.
world.createDistanceJoint({
  bodyA: anchor, bodyB: load,
  enableRigid: false, enableLimit: true,
  minLength: 0, maxLength: 4,
});

// فنر سیستم تعلیق.
world.createDistanceJoint({
  bodyA: chassis, bodyB: wheel,
  enableSpring: true, hertz: 4, dampingRatio: 0.7,
});
```

<div dir="rtl">

اجسام متصل به‌طور پیش‌فرض با هم برخورد نمی‌کنند؛ اگر می‌خواهید برخورد کنند
`collideConnected: true` بدهید.

---

## کوئری‌ها

</div>

```ts
// نزدیک‌ترین برخورد در مسیر پرتو.
const hit = world.rayCastClosest(0, 0, 10, 0);
if (hit) {
  console.log(hit.fixture.body.userData, hit.point.toFloats(), hit.normal.toFloats());
}

// همهٔ برخوردها، با کنترل کامل روی پیمایش.
world.rayCast(0, 0, 10, 0, (fixture, point, normal, fraction) => {
  if (fixture.isSensor) return -1;  // نادیده بگیر، بازهٔ فعلی را نگه دار
  hits.push(fixture);
  return fraction;                  // جست‌وجو را به برخوردهای نزدیک‌تر محدود کن
});

// هر چیزی که با یک کادر هم‌پوشانی دارد، یا شامل یک نقطه است.
world.queryAABB(-5, 0, 5, 10, (fixture) => { found.push(fixture); return true; });
world.queryPoint(mouseX, mouseY, (fixture) => { picked = fixture; return false; });
```

---

<div dir="rtl">

## اجسام پرسرعت

اجسامی که ممکن است در یک گام از دیوار رد شوند باید `bullet: true` داشته باشند تا
تست جاروبی (conservative advancement) روی آن‌ها اجرا شود:

</div>

```ts
const projectile = world.createBody({
  type: BodyType.Dynamic,
  position: { x: 0, y: 1 },
  linearVelocity: { x: 150, y: 0 },  // ۲٫۵ متر در هر گام با ۶۰ هرتز
  bullet: true,
});
projectile.addFixture({ shape: Circle.of(0.05), density: 5 });
```

<div dir="rtl">

فقط bulletها جارو می‌شوند، پس خاموش گذاشتن این پرچم هیچ هزینه‌ای ندارد. اجسام با
سرعت متوسط را همان تماس‌های احتمالی (speculative) پوشش می‌دهند.

---

## شبکه

Pulse2D هر سه قطعه‌ای که netcode از نوع lockstep و rollback لازم دارد را همراه
دارد. راهنمای کامل: **[NETWORKING.md](NETWORKING.md)**

</div>

```ts
import { saveSnapshot, loadSnapshot, checksumWorld, RollbackManager } from 'pulse2d';

// تشخیص desync: این عدد را رد و بدل و مقایسه کنید.
const digest = checksumWorld(world);

// ذخیره / بازیابی دستی.
const snap = saveSnapshot(world);
loadSnapshot(world, snap);

// یا بگذارید درایور rollback پیش‌بینی و بازشبیه‌سازی را انجام دهد.
const rb = new RollbackManager(world, {
  maxRollbackFrames: 12,
  applyInputs: (tick, inputs) => applyToGame(tick, inputs),
});

rb.addPlayer(localId);
rb.addPlayer(remoteId);

rb.addLocalInput(localId, readInput());
rb.advance();                                  // یک گام دنیا را جلو می‌برد

socket.on('input', ({ tick, playerId, input }) => {
  rb.addRemoteInput(tick, playerId, input);    // در صورت پیش‌بینی غلط، خودکار rollback می‌کند
});
```

<div dir="rtl">

> برای هر چیزی که روی شبیه‌سازی اثر دارد به‌جای `Math.random()` از `world.rng`
> استفاده کنید. این مولد seed دارد، در snapshot ذخیره می‌شود و درست بازگردانده
> می‌شود.

### ادعای قطعیت چطور تضمین می‌شود

نوشتن «نتیجهٔ یکسان روی هر دستگاه» در README آسان است و درست نگه‌داشتنش سخت؛ به
همین دلیل این ادعا در Pulse2D یک **تست** است، نه یک وعده. فایل
`test/golden.test.mjs` چهار صحنه را بازپخش می‌کند — به‌هم‌ریختن ۶۰ جسم، همهٔ
انواع مفصل با هم، CCD گلوله‌ای، و هرمی که می‌خوابد و دوباره بیدار می‌شود — و
checksum **هر تیک** را در یک چکیدهٔ واحد می‌آمیزد. این چکیده‌ها در
`test/golden.json` ثبت شده‌اند و CI آن‌ها را روی لینوکس، ویندوز و مک‌اواس، روی
معماری‌های x64 و arm64، از Node 18 تا 24، و برای هر دو بک‌اند عددی بازپخش
می‌کند.

اینکه به‌جای حالت نهایی، کل **مسیر حرکت** چکیده می‌شود اهمیت دارد: بیشتر صحنه‌ها
ته‌نشین می‌شوند و به خواب می‌روند، پس یک واگرایی در تیک ۲۰۰ می‌توانست پیش از
تیک آخر مستهلک شود و دیده نشود.

همین بررسی را می‌توانید روی سخت‌افزار خودتان اجرا کنید:

</div>

```bash
node scripts/golden.mjs          # چکیده‌های این ماشین را چاپ می‌کند
node scripts/golden.mjs --check  # آن‌ها را با قرارداد ثبت‌شده مقایسه می‌کند
```

<div dir="rtl">

اگر این دستور روی پلتفرم هدف شما پاس شد، Pulse2D با هر پلتفرم دیگری که آن را
پاس کند نتیجهٔ یکسان می‌دهد.

---

## رندر دیباگ

</div>

```ts
import { DebugDraw } from 'pulse2d';

const draw = new DebugDraw(canvas.getContext('2d'), { pixelsPerMeter: 32 });
draw.flags.contacts = true;
draw.flags.stats = true;

function frame() {
  world.step();
  draw.begin();       // تبدیل جهان→صفحه را نصب می‌کند (محور y+ رو به بالا)
  draw.drawWorld(world);
  draw.end();
  requestAnimationFrame(frame);
}
```

<div dir="rtl">

لایه‌ها: `shapes`، `fill`، `joints`، `contacts`، `contactNormals`،
`contactImpulses`، `aabbs`، `centerOfMass`، `sleepState`، `velocities`،
`stats`. هیچ بخشی از شبیه‌سازی به رندرر وابسته نیست، پس اگر importش نکنید از
باندل نهایی حذف می‌شود.

---

## کارایی

اندازه‌گیری روی Node 20، معماری x64، تک‌هسته. میانهٔ ۳۰۰ گام پس از گرم شدن.

| سناریو | میانهٔ هر گام | درصد از فریم ۶۰ هرتز |
|---|---:|---:|
| هرم، ۲۱۰ جعبه (۵۸۸ تماس) | ۱٫۴۶ ms | ۹٪ |
| هرم، ۴۶۵ جعبه (۱۳۲۴ تماس) | ۳٫۴۵ ms | ۲۱٪ |
| ۵۰۰ دایرهٔ در حال سقوط | ۳٫۳۰ ms | ۲۰٪ |
| ۱۰۰۰ دایرهٔ در حال سقوط | ۷٫۳۸ ms | ۴۴٪ |
| ۱۰۰۰ شکل ترکیبی در حال سقوط | ۸٫۰۳ ms | ۴۸٪ |
| ۱۰۰۰ جسم، همه خواب | ۱٫۳۶ ms | ۸٪ |
| ۳۰۰ مفصل (۳۰ زنجیرهٔ ragdoll) | کمتر از ۰٫۰۱ ms | ۰٪ |

| عملیات شبکه | هزینه |
|---|---:|
| `saveSnapshot` (۵۰۰ جسم) | ۰٫۲۲ ms، ۷۱ کیلوبایت |
| `loadSnapshot` (۵۰۰ جسم) | ۳٫۸۴ ms |
| `checksumWorld` (۵۰۰ جسم) | ۰٫۳۹ ms |
| ۱ ثانیه تاریخچهٔ rollback با ۶۰ هرتز | ۴٫۲ مگابایت |

برای بازتولید: `npm run bench`

**تنظیم.** اهرم اصلی `subSteps` است (پیش‌فرض `۴`). گام‌های فرعی بیشتر یعنی
پشته‌های سفت‌تر و مدیریت بهتر حرکت سریع، با هزینهٔ خطی. ترجیحاً این را بالا ببرید
نه `velocityIterations` را.

</div>

```ts
const world = new World({
  subSteps: 8,             // سفت‌تر، حدود ۲ برابر هزینهٔ حل
  velocityIterations: 2,   // تکرارهای بایاس‌دار در هر گام فرعی
  relaxIterations: 1,      // حذف بیش‌روی بایاس
});
```

---

<div dir="rtl">

## مستندات

| سند | محتوا |
|---|---|
| **[API.md](API.md)** | مرجع کامل همهٔ کلاس‌ها و توابع عمومی |
| **[DETERMINISM.md](DETERMINISM.md)** | قطعیت چطور به دست آمده، چه چیزی خرابش می‌کند، و قوانینی که باید رعایت کنید |
| **[NETWORKING.md](NETWORKING.md)** | یکپارچه‌سازی lockstep و rollback، دیباگ desync |
| **[ARCHITECTURE.md](ARCHITECTURE.md)** | درون موتور: خط لولهٔ گام، ریاضیات حل‌کننده، فاز گسترده |
| **[RECIPES.md](RECIPES.md)** | راه‌حل‌های آماده: کنترلر کاراکتر، سکوی یک‌طرفه، خودرو، انفجار |
| **[CONTRIBUTING.md](../../CONTRIBUTING.md)** | راه‌اندازی توسعه، قوانین قطعیت که مشارکت‌کننده باید رعایت کند، چک‌لیست PR |
| **[CHANGELOG.md](../../CHANGELOG.md)** | همهٔ نسخه‌ها، و اینکه کدام‌یک نتیجهٔ شبیه‌سازی را تغییر داده‌اند |
| **[SECURITY.md](../../SECURITY.md)** | مدل تهدید — به‌ویژه اینکه چرا snapshot مرز اعتماد نیست |
| **[RELEASING.md](RELEASING.md)** | فرایند انتشار: تنظیمات گیت‌هاب، شماره‌گذاری نسخه، tag زدن |

هر فایل منبع هم یک توضیح در سطح ماژول دارد که می‌گوید **چرا** این‌طور کار
می‌کند، نه فقط چه کاری می‌کند.

---

## ساختار پروژه

</div>

```
src/
  math/          Vec2، Rot، Transform، Mat22 · مثلثات و RNG قطعی
    scalar.ts        انتخابگر بک‌اند  ← این را عوض کنید تا نوع عدد تغییر کند
    scalar.f64.ts    بک‌اند IEEE-754 دقت مضاعف (پیش‌فرض)
    scalar.fixed.ts  بک‌اند ممیزثابت Q16.16
  collision/     شکل‌ها، AABB، فاصلهٔ GJK، فاز باریک SAT، فاز گستردهٔ BVH
  dynamics/      Body، Fixture، Contact، Solver، World، برخورد پیوسته
    joints/          شش نوع مفصل روی یک کلاس پایهٔ مشترک
  net/           snapshot، checksum، درایور rollback
  render/        رندرر دیباگ Canvas (اختیاری)
  util/          ثابت‌های تنظیم
```

<div dir="rtl">

همهٔ ماژول‌ها بدون اثر جانبی هستند و مستقل قابل import.

---

## ساخت از سورس

</div>

```bash
npm install
npm test          # ۲۶۰ تست (اول build می‌کند — یادداشت زیر)
npm run build     # باندل‌ها و فایل‌های تایپ در dist/
npm run bench     # مجموعهٔ سنجش کارایی
npm run check     # بررسی تایپ بدون خروجی
npm run demo      # دموی تعاملی روی http://localhost:8080

node scripts/golden.mjs --check   # بررسی قرارداد قطعیت روی این ماشین
```

> **تست‌ها روی `dist/` اجرا می‌شوند نه `src/`**، تا همان باندلی را بیازمایند که
> بازی شما import می‌کند. به همین دلیل `npm test` با هوک `pretest` اول build
> می‌کند — نیازی به مرحلهٔ جداگانه نیست.

### دمو

با `npm run demo` یک محیط تعاملی بالا می‌آید با **۱۶ صحنه** (هرم، دومینو، پل
طنابی، پلینکو، درام چرخان، بیلیارد، قلعهٔ تخریب‌شدنی، جسم نرم، نوار نقاله،
ماشین با سیستم تعلیق فنری، تست فشار ۸۰۰ جسمی و…) و **۶ ابزار** — کشیدن،
ساختن جسم، توپخانه، انفجار و ray cast زنده.

گام‌های فرعی، گرانش، خوابیدن و warm starting همه حین اجرا قابل تنظیم‌اند، و یک
دکمه صحنهٔ فعلی را دو بار با seed یکسان اجرا می‌کند تا نشان دهد checksumها در
تک‌تک tickها یکسان‌اند.

<div dir="rtl">

خروجی build شامل ESM، CJS و UMD برای هر دو بک‌اند به‌همراه فایل‌های `.d.ts` است.

---

## پیش‌نیازها

هر محیط ES2020: کروم ۹۰+، فایرفاکس ۹۰+، سافاری ۱۵+، Node 18+.
بدون وابستگی، بدون WebAssembly، بدون تولید کد در زمان build.

روی Node 20 و Node 24، در لینوکس و ویندوز (PowerShell) آزموده شده است.

---

## مجوز

MIT

</div>
