<div dir="rtl">

# شبکه

ساخت مولتی‌پلیر قطعی روی Pulse2D: lockstep، rollback، تشخیص و دیباگ desync.

اول [DETERMINISM.md](DETERMINISM.md) را بخوانید — قوانین آنجا همان چیزی هستند که
باعث می‌شوند هیچ‌کدام از این‌ها کار کند.

[English version](../NETWORKING.md)

---

## ۱. مدل

netcode قطعی **ورودی** می‌فرستد، نه وضعیت. همهٔ همتاها همان شبیه‌سازی را اجرا
می‌کنند و مستقلاً به نتیجهٔ یکسان می‌رسند.

|  | چه می‌فرستد | پهنای باند | حس تأخیر | تقلب |
|---|---|---|---|---|
| **همگام‌سازی وضعیت** | موقعیت همهٔ اشیاء | متناسب با تعداد اشیاء | سرور مرجع است | سخت |
| **Lockstep** | فقط ورودی | خیلی کم، ثابت | منتظر کندترین همتا | نیاز به دقت |
| **Rollback** | فقط ورودی | خیلی کم، ثابت | پاسخ محلی آنی | نیاز به دقت |

ورودی یک بازی دو نفره چند بایت در هر tick است، فارغ از اینکه دنیا ده شیء دارد یا
ده هزار تا. جذابیت کار همین است — و فقط وقتی جواب می‌دهد که شبیه‌سازی بیت‌به‌بیت
یکسان باشد، که تضمین Pulse2D همین است.

---

## ۲. Lockstep

ساده‌ترین مدل درست: هر همتا صبر می‌کند تا **همهٔ** ورودی‌های tick شمارهٔ *N* را
داشته باشد، بعد گام برمی‌دارد.

</div>

```ts
import { World, checksumWorld } from 'pulse2d';

const world = new World({ gravity: { x: 0, y: -10 }, seed: MATCH_SEED });
buildLevel(world);                       // روی همهٔ همتاها یکسان

const inputBuffer = new Map();           // tick -> Map<playerId, Input>
const INPUT_DELAY = 3;                   // چند tick فاصلهٔ زمان‌بندی

function tryStep() {
  const tick = world.tick;
  const frame = inputBuffer.get(tick);
  if (!frame || frame.size < playerCount) return false;   // هنوز منتظریم

  for (const [playerId, input] of frame) applyInput(playerId, input);
  world.step();
  inputBuffer.delete(tick);
  return true;
}

// ورودی محلی را برای چند فریم جلوتر بفرست تا به‌موقع برسد.
function sendLocalInput() {
  const targetTick = world.tick + INPUT_DELAY;
  const input = readLocalInput();
  socket.send({ tick: targetTick, playerId: myId, input });
  record(targetTick, myId, input);       // برای خودمان هم اعمال کن
}
```

<div dir="rtl">

**مزایا:** استدلال دربارهٔ آن بسیار ساده است، هیچ ماشین‌آلات rollback لازم ندارد.
**معایب:** هر بازیکن تأخیر بدترین بازیکن را حس می‌کند. برای RTS، بازی نوبتی و
co-op خوب است؛ برای بازی مبارزه‌ای یا اکشن خیلی کند است.

مقدار `INPUT_DELAY` برای رسیدن بسته‌ها وقت می‌خرد. سه tick با ۶۰ هرتز یعنی
۵۰ میلی‌ثانیه فرصت، به قیمت ۵۰ میلی‌ثانیه تأخیر ورودی.

---

## ۳. Rollback

دنیا همیشه روی tick محلی اجرا می‌شود و ورودی‌های راه دور را **پیش‌بینی** می‌کند.
وقتی ورودی واقعی می‌رسد و با پیش‌بینی نمی‌خواند، دنیا عقب می‌رود و دوباره
شبیه‌سازی می‌شود. ورودی محلی آنی اعمال می‌شود، پس بازی فارغ از پینگ پاسخگو
حس می‌شود.

کلاس `RollbackManager` کل این حلقه را پیاده کرده است:

</div>

```ts
import { World, RollbackManager } from 'pulse2d';

const world = new World({ gravity: { x: 0, y: -10 }, seed: MATCH_SEED });
const players = buildLevel(world);

const rb = new RollbackManager(world, {
  maxRollbackFrames: 12,                 // حدود ۲۰۰ میلی‌ثانیه تاریخچه با ۶۰ هرتز

  // یک بار در هر tick اجرا می‌شود، هم در پیشروی عادی و هم در بازشبیه‌سازی.
  // نسبت به هر چیزی بیرون از دنیا باید خالص باشد.
  applyInputs(tick, inputs) {
    for (const [playerId, input] of inputs) {
      const body = players[playerId];
      if (input.left)  body.applyLinearImpulse(-8, 0);
      if (input.right) body.applyLinearImpulse( 8, 0);
      if (input.jump && onGround(body)) body.applyLinearImpulse(0, 40);
    }
  },

  // «تکرار آخرین ورودی» برای بیشتر بازی‌ها انتخاب درستی است.
  predictInput: (tick, playerId, last) => last ?? { left: false, right: false, jump: false },

  onRollback: (from, to, frames) => stats.record(frames),
});

rb.addPlayer(localId);
rb.addPlayer(remoteId);

// --- هر فریم ---
const input = readLocalInput();
rb.addLocalInput(localId, input);
socket.send({ tick: rb.tick, playerId: localId, input });
rb.advance();                            // snapshot، اعمال ورودی، یک گام
render();

// --- هر وقت شبکه چیزی تحویل داد ---
socket.on('input', ({ tick, playerId, input }) => {
  rb.addRemoteInput(tick, playerId, input);   // در پیش‌بینی غلط خودکار rollback می‌کند
});
```

<div dir="rtl">

### متد `advance()` چه می‌کند

۱. یک snapshot از وضعیت پیش از گام ذخیره می‌کند.
۲. ورودی‌ها را جمع می‌کند: محلی‌ها معتبرند؛ راه دورهای غایب پیش‌بینی و علامت‌گذاری
   می‌شوند.
۳. رکورد `{ tick, snapshot, inputs, predicted }` را در حلقهٔ تاریخچه می‌گذارد.
۴. `applyInputs` را صدا می‌زند و سپس `world.step()`.

### متد `addRemoteInput()` چه می‌کند

اگر آن tick قبلاً شبیه‌سازی شده و ورودی پیش‌بینی (یا غایب) بوده، پیش‌بینی با
واقعیت مقایسه می‌شود. در صورت عدم تطابق، snapshot **قبل** از آن tick بازیابی
می‌شود و همهٔ tickهای بعدی با ورودی‌های اصلاح‌شده بازپخش می‌شوند.

> **بازیکنانتان را ثبت کنید.** در شروع مسابقه `addPlayer(id)` را صدا بزنید. بدون
> آن، اولین ورودی یک بازیکن قابل پیش‌بینی نیست — آن tick بدون هیچ ورودی
> شبیه‌سازی می‌شود و چون چیزی به‌عنوان پیش‌بینی علامت نخورده، این اشتباه هرگز
> قابل rollback نخواهد بود.

### خلوص `applyInputs`

تابع `applyInputs` برای هر tick بازپخش‌شده دوباره صدا زده می‌شود، پس نباید به
چیزی بیرون از دنیا دست بزند:

</div>

```ts
// ✗ این‌ها در هر rollback دوباره اجرا می‌شوند
applyInputs(tick, inputs) {
  playSound('jump');            // صدای لرزان و تکراری
  score += 10;                  // امتیاز باد می‌کند
  spawnParticles();             // زبالهٔ بصری
}

// ✓ نیت را ثبت کن؛ بعد از قطعی شدن tick عمل کن
applyInputs(tick, inputs) {
  if (input.jump) { body.applyLinearImpulse(0, 40); pendingEvents.push({ tick, kind: 'jump' }); }
}
```

<div dir="rtl">

آرایهٔ `pendingEvents` را فقط برای tickهای قدیمی‌تر از `rb.oldestTick` تخلیه
کنید، چون دیگر قابل rollback نیستند.

---

## ۴. هزینه و بودجه

اندازه‌گیری روی Node 20، معماری x64، با ۵۰۰ جسم:

| عملیات | هزینه |
|---|---|
| `saveSnapshot` | ۰٫۲۲ ms، ۷۱ کیلوبایت |
| `loadSnapshot` | ۳٫۸۴ ms |
| `checksumWorld` | ۰٫۳۹ ms |
| `world.step()` (۵۰۰ دایره) | ۳٫۳۰ ms |

یک rollback به عمق *n* فریم تقریباً `loadSnapshot + n × (step + saveSnapshot)`
هزینه دارد. برای دنیای ۵۰۰ جسمی با عقب‌گرد ۶ فریمی: `۳٫۸ + ۶ × ۳٫۵ ≈ ۲۵ ms` —
برای یک فریم با ۶۰ هرتز زیاد است.

راهنمای عملی:

- **دنیای شبیه‌سازی‌شده را کوچک نگه دارید.** بازی‌های rollback معمولاً فقط چیزی
  را شبیه‌سازی می‌کنند که روی گیم‌پلی اثر دارد (۲ تا ۴ کاراکتر، چند پرتابه) و
  بقیهٔ صحنه را دکور غیرشبیه‌سازی‌شده می‌گذارند.
- **برای تاریخچه بودجه بگذارید.** ۷۱ کیلوبایت × ۶۰ = ۴٫۲ مگابایت برای هر ثانیه
  تاریخچه با ۵۰۰ جسم. مقدار `maxRollbackFrames: 12` حدود ۸۵۰ کیلوبایت است. با
  `rb.historyBytes` بررسی کنید.
- **عمق rollback را محدود کنید.** بیش از حدود ۸ فریم، بهتر است ۱ تا ۲ tick تأخیر
  ورودی بگذارید تا فراوانی پیش‌بینی غلط کم شود.
- **کمتر snapshot بگیرید.** برای دنیاهای بزرگ، هر *k* تیک یک snapshot بگیرید و
  در rollback تا *k* تیک اضافه بازپخش کنید — ارزان‌تر در حافظه، گران‌تر در CPU.

---

## ۵. تشخیص desync

از روز اول checksum را راه بیندازید. desyncی که در توسعه پیدا شود یک بعدازظهر
وقت می‌برد؛ همان در محصول یک هفته.

</div>

```ts
import { ChecksumLog } from 'pulse2d';

const log = new ChecksumLog(512);

// هر tick
world.step();
const digest = log.recordWorld(world);

// هر حدود ۳۰ tick، digestها را رد و بدل کنید
if (world.tick % 30 === 0) socket.send({ type: 'sum', tick: world.tick, digest });

socket.on('sum', ({ tick, digest }) => {
  const mine = log.get(tick);
  if (mine !== undefined && mine !== digest) {
    console.error(`DESYNC در tick ${tick}: محلی ${mine.toString(16)} راه دور ${digest.toString(16)}`);
    captureDebugState(tick);
  }
});
```

<div dir="rtl">

متد `ChecksumLog.findDivergence(remoteMap)` **اولین** tick متفاوت را از یک دسته
برمی‌گرداند. فقط همان اولی مهم است؛ بقیه نویز پایین‌دستی‌اند.

---

## ۶. دیباگ کردن یک desync

۱. **اولین tick خراب را پیدا کنید** — با `findDivergence`، نه tickی که مشکل را
   در آن دیدید.
۲. **snapshotهای `tick - 1` را مقایسه کنید** روی هر دو همتا.
   - *snapshotها یکسان* ← واگرایی از **ورودی‌ها** آمده. تحویل، ترتیب و کوانتیزه
     کردن ورودی را بررسی کنید.
   - *snapshotها متفاوت* ← واگرایی در **شبیه‌سازی** است و زودتر از آنچه فکر
     می‌کنید شروع شده. عقب‌تر بروید.
۳. **جسم‌به‌جسم diff بگیرید.** مقادیر `id، موقعیت، چرخش، سرعت` را برای همهٔ اجسام
   چاپ کنید و اولین تفاوت را پیدا کنید. بعد بپرسید چه چیزی با آن در تماس بوده.
۴. **هش را تفکیک کنید.** فراخوان `checksumWorld(world, true)` فقط موقعیت‌ها را
   پوشش می‌دهد. اگر فقط-موقعیت یکسان بود ولی کامل نه، رانش در ضربه‌های انباشتهٔ
   تماس است نه در وضعیت قابل مشاهده.
۵. **محلی بازتولید کنید.** کل جریان ورودی را ضبط کنید و دو بار در یک پروسه
   بازپخش کنید. اگر همان‌جا واگرا شد، باگ در کد بازی شماست نه در ممیز شناور
   بین‌پلتفرمی — قوانین [DETERMINISM.md بخش ۴](DETERMINISM.md#۴-قوانین-کد-بازی-شما)
   را ببینید.

### متهمان همیشگی

| نشانه | علت |
|---|---|
| بلافاصله در tick صفر واگرا می‌شود | پیکربندی متفاوت دنیا، ترتیب متفاوت ساخت مرحله، یا بازیکن ثبت‌نشده |
| فقط با ۳ بازیکن یا بیشتر واگرا می‌شود | ترتیب ساخت اشیاء به زمان رسیدن بسته وابسته است |
| فقط بعد از یک rollback واگرا می‌شود | `applyInputs` خالص نیست، یا وضعیت بازی بیرون از دنیا نگهداری می‌شود |
| فقط بین مرورگرهای مختلف واگرا می‌شود | یک فراخوانی `Math.sin`/`Math.random`/`Date.now` در کد بازی |
| بعد از چند دقیقه واگرا می‌شود | خطای انباشته که از یک مرز مقایسه عبور کرده — معمولاً ورودی کوانتیزه‌نشده |

---

## ۷. کوانتیزه کردن ورودی

ورودی‌های آنالوگ باید **قبل** از ارسال کوانتیزه شوند و همان مقدار کوانتیزه‌شده
محلی هم اعمال شود. وگرنه فرستنده با دقت کامل شبیه‌سازی می‌کند و گیرنده با هر
چیزی که از سریال‌سازی جان سالم به در برده.

</div>

```ts
const QUANT = 1024;                                // تفکیک ۱/۱۰۲۴ متر
const q = (v) => Math.round(v * QUANT);            // برای شبکه
const dq = (v) => v / QUANT;                       // برای شبیه‌سازی

const input = { aimX: q(mouseWorldX), aimY: q(mouseWorldY), buttons: bitmask };

socket.send(input);
applyLocally(input);                               // همان اعداد صحیح همتای راه دور

function applyLocally({ aimX, aimY }) {
  turret.setTarget(dq(aimX), dq(aimY));
}
```

<div dir="rtl">

به‌جای فرستادن بولین، دکمه‌ها را در یک bitmask بگذارید — کوچک‌تر است و از تفاوت
ترتیب کلیدهای JSON مصون می‌ماند.

---

## ۸. پیوستن وسط مسابقه

snapshot **وضعیت** را ذخیره می‌کند نه **ساختار** را، پس بازیکن دیررسیده به هر دو
نیاز دارد:

</div>

```ts
// میزبان
socket.send({
  type: 'sync',
  tick: world.tick,
  level: serialiseLevel(),               // توصیف اجسام/فیکسچرها به روش خودتان
  snapshot: {
    tick: snap.tick,
    data: Array.from(snap.data),
    meta: Array.from(snap.meta),
  },
  rngState: world.rng.getState(),
});

// پیونده
const world = new World({ gravity: HOST_GRAVITY, seed: MATCH_SEED });
rebuildLevel(world, msg.level);          // باید شناسهٔ اجسام یکسانی تولید کند
loadSnapshot(world, {
  tick: msg.snapshot.tick,
  data: new Float64Array(msg.snapshot.data),
  meta: new Int32Array(msg.snapshot.meta),
});
world.rng.setState(...msg.rngState);
```

<div dir="rtl">

پیونده باید اجسام را به **همان ترتیب** میزبان بسازد تا شناسه‌ها بخوانند. بلافاصله
با تبادل checksum بررسی کنید، قبل از اینکه اجازهٔ ورودی بدهید.

برای انتقال باینری، مستقیماً `snap.data.buffer` و `snap.meta.buffer` را بفرستید
به‌جای تبدیل از طریق آرایه.

---

## ۹. ایمنی نسخه

مقدار `PROTOCOL_VERSION` و شناسهٔ بک‌اند عددی در هدر هر snapshot درج می‌شوند و
`loadSnapshot` در صورت عدم تطابق خطا می‌دهد، به‌جای خواندن بی‌صدای اشتباه:

</div>

```
Pulse2D: protocol mismatch (snapshot 1, build 2)
Pulse2D: scalar backend mismatch (fixed-point vs float)
```

```ts
import { PROTOCOL_VERSION, VERSION, Scalar } from 'pulse2d';

socket.send({ type: 'hello', protocol: PROTOCOL_VERSION, version: VERSION, backend: Scalar.BACKEND });
```

<div dir="rtl">

هر وقت یک ثابت تنظیم در `util/settings.ts`، جزئیاتی از حل‌کننده یا نسخهٔ Pulse2D
را عوض کردید — یعنی هر چیزی که نتیجهٔ شبیه‌سازی را تغییر می‌دهد —
`PROTOCOL_VERSION` را بالا ببرید.

---

## ۱۰. چک‌لیست

پیش از انتشار:

- [ ] هیچ `Math.random` در کد شبیه‌سازی نیست — از `world.rng` استفاده شده
- [ ] هیچ `Date.now` / `performance.now` شبیه‌سازی را نمی‌راند — از `world.tick` استفاده شده
- [ ] همهٔ ورودی‌های آنالوگ قبل از ارسال کوانتیزه شده و همان مقادیر محلی اعمال می‌شوند
- [ ] همهٔ بازیکنان قبل از اولین tick با `addPlayer` ثبت شده‌اند
- [ ] `applyInputs` خالص است؛ صدا، ذرات و امتیاز به تعویق افتاده‌اند
- [ ] اجسام روی همهٔ همتاها به ترتیب قطعی ساخته و حذف می‌شوند
- [ ] پیکربندی دنیا (گرانش، `timeStep`، `subSteps`، تکرارها) یکسان است
- [ ] `PROTOCOL_VERSION` هنگام اتصال بررسی می‌شود
- [ ] checksumها رد و بدل و ثبت می‌شوند
- [ ] بازپخش دوبارهٔ یک جریان ورودی ضبط‌شده checksum یکسان می‌دهد

</div>
