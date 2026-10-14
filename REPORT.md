MSSV: 202602571

# Lab 11 · Scenario Forge — báo cáo

Họ tên: Nguyễn Trần Kiên · MSSV ghi trên lớp: 2A202602571 (ghi dạng số `202602571` ở dòng đầu để `run_all.py` đọc được).

Thị trấn của bạn (`meta.json: _run.town`): **D** (`p_bin` = (wet 0,15 · pedLook 0,90 · brakes 0,20), `cut_gap_median` = 14 m). Seed: `meta.json: seeds = {mc: 3354474268, search: 3699743555, test: 4018214513}`. Mọi bước trong `meta.json: steps` có `quick = false`. Mọi số dưới đây trích từ `evidence/` (tên file: trường); vài số *tính tay* từ các số đó được ghi rõ là "tính từ".

## 1. Phát lại log có trung thực không? (bước A)

| `replay.json` | giá trị |
|---|---|
| `n` / `dropped_human_crash_logs` / `n_used` | 400 / 0 / 400 (người lái thật không gây va chạm trong 400 log, nên không log nào bị bỏ) |
| `replay.crashes` theo `contact` | **108** = `rear` 100 + `side` 8 |
| `reactive.crashes` | **0** |
| `crash_in_both` | 0 |

Trên cùng 400 kịch bản, planner trong thế giới **phản ứng** không va chạm lần nào, nhưng chạy trên **log phát lại** thì 108/400 (27 %) kết thúc bằng va chạm, 100 trong đó là `rear`: xe bám đuôi (TG) đâm vào đuôi xe ta. Lý do: planner **phanh sớm hơn và dừng hẳn** ở chỗ người lái thật chỉ đi chậm lại: khi xe buýt che vạch, planner vào chế độ thận trọng (`cautionV` = 7,5 m/s trong 55 m trước vạch) rồi nhường người đi bộ (`pedBuffer`), còn người lái (HUMAN, `T` = 1,1 s, `b` = 3,5) đi ~10 m/s qua vạch trước khi người bước xuống. Trong log, TG đã được ghi là bám theo *người lái thật*; khi phát lại, TG lặp lại đúng gia tốc đã ghi, **không biết** phía trước giờ là một planner đang phanh sớm hơn, nên tông vào. Đó là va chạm **giả**: ngoài đời TG là người, sẽ thấy xe ta chậm lại và phanh theo; thế giới phản ứng (IDM) mô phỏng đúng điều đó và cho 0 va chạm. `world.json` cũng kể câu chuyện này: `replay_events` = [ego|TG, t = 14,8 s] trong khi `reactive_events` = [].

Kịch bản `replay.json: replay_only[0]` (`scenario` = 4, `contact` = rear, `min_ttc_reactive` = 2,85 s; tham số `tgT` = 1,57 s, `pedT0` = 8,1 s, `pedGap` = 3,55 s, `pedLook` = 1): chạy lại trong trình xem (R-01) thấy từ ~9,5 s xe buýt che vạch nên planner giảm còn ~7,2 m/s (mode 2), 15 s thấy P1 bước xuống nên phanh và dừng ở x ≈ 114 m trước vạch (mode 3); người lái thật trong log đi 9–10 m/s và qua vạch lúc ~14,7 s, trước khi P1 xuống đường. TG phát lại chỉ giảm tốc theo người lái (10 → 4 m/s) chứ không theo xe ta đang đứng, nên tông vào đuôi xe ta lúc 17,8 s. Trong thế giới phản ứng TTC nhỏ nhất của xe ta chỉ xuống 2,85 s, hoàn toàn an toàn. Ảnh: `evidence/shots/q1_replay_only_R-01_rear.png` (R-01, 17,8 s, xe bám đuôi tông đuôi).

## 2. Kịch bản bất khả thi (bước B)

`lab11/feasible.py: is_feasible` loại một kịch bản nếu **bất kỳ** điều nào sau đây xảy ra:

| Luật | Căn cứ vật lý |
|---|---|
| gia tốc ngang khi chuyển làn (π/`cutDur`)²·3,5/2 > μ·g | Đổi làn 3,5 m theo quỹ đạo cosin: gia tốc ngang đỉnh = (π/T)²·A/2; lốp chỉ bám được μ·g (khô 0,8·9,81 ≈ 7,8 m/s², ướt 0,5·9,81 ≈ 4,9 m/s²). Đổi làn trong 1,2 s cần 12 m/s² ngang: xe trượt, không ai làm được. |
| `cutBrake` > μ·g | Giảm tốc dọc cũng bị giới hạn ma sát lốp–đường; phanh 8,5 m/s² trên đường ướt (4,9) là bịa. |
| `pedV` > 2,5 m/s | Trên 2,5 m/s là chạy, không còn là "người đi bộ"; dữ liệu tốc độ đi bộ 15th–85th percentile nằm trong 0,9–1,6 m/s (TRL PPR700). |
| `cutV` ∉ (0, 22] m/s | Xe chuyển làn đứng yên hoặc lùi (≤ 0) không "cắt làn"; trên 22 m/s (79 km/h) vượt giới hạn phố nội đô của thế giới này. |

Trong `hunt.json: <mode>.infeasible`: `crosswalk_block` = **11** (nhiều nhất), `bus_bypass` = 9, `rear_ended` = 7, `cut_in` = 2 (Monte Carlo tự nhiên: `mc.json: infeasible` = 12/3000). Cuộc săn `crosswalk_block` bị kéo về phía đó vì điểm nguy hiểm của nó là `ped_room` = khoảng dừng còn lại = (khoảng cách tới người) − v²/(2μg): đường **ướt** (μ = 0,5) làm quãng phanh dài gấp 1,6 lần, nên elite của CEM dồn về `wet = 1` (`hunt.json: crosswalk_block.proposal.p_bin[0]` = 0,88 so với 0,15 tự nhiên; `pedV` ngược lại bị kéo về người đi **chậm**, `proposal.mean[5]` = −2,40, tức ở lâu trong làn). Đường ướt lại siết luật khả thi thứ nhất: với μ = 0,5 chuyển làn cần `cutDur` > 1,88 s thay vì 1,48 s, trong khi q vẫn để `cutDur` gần phân phối tự nhiên (`proposal.mean[1]` ≈ 0), nên đuôi ngắn của `cutDur` rơi vào vùng bất khả thi nhiều hơn các cuộc săn khác. (Kiểm tra bằng cách lấy mẫu lại từ proposal cuối: mọi mẫu bị loại đều do luật gia tốc ngang, không do `pedV` hay `cutBrake`.) Nếu **đếm** chúng là va chạm, ước lượng IS bị thổi lên bởi những kịch bản không người thật nào tạo ra (w·y > 0 ở chỗ p(u) vẫn > 0 nhưng thế giới thật không có), và CEM còn co nhanh hơn vào vùng vô lý đó: con số báo là xác suất của một thế giới không tồn tại, còn các kiểu thật sự xảy ra (ví dụ `bus_bypass`) bị bỏ sót.

## 3. Monte Carlo cho bạn biết gì (bước C1)

`mc.json`: `n` = **3000**, `crashes` = **2**, `p` = **6,67·10⁻⁴**, Wilson 95 % [`lo`, `hi`] = **[1,83·10⁻⁴, 2,43·10⁻³]**, `modes` = {bus_bypass: 2}.

Khoảng Wilson rộng gấp ~13 lần ở hai đầu. Để nửa khoảng còn ±20 % của p: 1,96·√(p(1−p)/n) = 0,2·p ⇒ n = (1,96/0,2)²·(1−p)/p ≈ 96/p; với p̂ = 6,67·10⁻⁴ thì n ≈ **1,44·10⁵** lần chạy (tính từ `mc.json: p`), tức gấp 48 lần ngân sách 3 000 hiện có, và cũng là ~96 va chạm quan sát được.

Kiểu **chưa hề xuất hiện** trong 3 000 lần: `cut_in`, `crosswalk_block`, `rear_ended` và `other` (`mc.json: per_mode.*.k` = 0). Con số đúng cho chúng **không phải 0** mà là cận trên Wilson với k = 0: `mc.json: per_mode.crosswalk_block.hi` = `per_mode.cut_in.hi` = `per_mode.rear_ended.hi` = `per_mode.other.hi` = **1,28·10⁻³** (lo = 0). Nghĩa là Monte Carlo chỉ nói được "mỗi kiểu đó hiếm hơn 1,3 trên nghìn", không nói được "không có".

## 4. Importance sampling, ESS và độ suy biến (bước C3)

`estimate.json`, n = 600 mẫu mỗi kiểu mỗi seed, q = 0,8·CEM + 0,2·tự nhiên:

| kiểu | seed | `p` | `hits` | `ess` | `top_share` | ghi chú |
|---|---|---|---|---|---|---|
| bus_bypass | 0 | 4,64·10⁻³ | 22 | 129 | **0,99** | một mẫu mang 99 % ước lượng |
| bus_bypass | 1 | 3,87·10⁻⁴ | 31 | 124 | 0,20 | |
| bus_bypass | 2 | 1,98·10⁻⁴ | 22 | 138 | 0,26 | |
| cut_in | 0/1/2 | 0 (hits 0) | 0 | 124 / 108 / 128 | — | báo `upper_zero_hits` = **2,5·10⁻²** |
| crosswalk_block | 0 | 4,91·10⁻⁷ | 97 | 123 | **0,55** | |
| crosswalk_block | 1 | 2,38·10⁻⁷ | 94 | 118 | **0,61** | |
| crosswalk_block | 2 | 4,65·10⁻⁷ | 97 | 132 | **0,77** | |
| rear_ended | 0/1/2 | 0 (hits 0) | 0 | 110 / 121 / 114 | — | báo `upper_zero_hits` = **2,5·10⁻²** |
| `other` | — | 0 (k = 0/3000, `source` = mc.json) | | | | `other.hi` = 1,28·10⁻³ |
| **total** | 0 / 1 / 2 | 4,64·10⁻³ / 3,88·10⁻⁴ / 1,99·10⁻⁴ | | | | trung vị **3,88·10⁻⁴** (tính từ 3 `seeds[].total`) |

**Ba seed có đồng ý không?** `crosswalk_block` đồng ý (2–5·10⁻⁷, cùng bậc). `bus_bypass` thì không: seed 0 cao gấp 12–23 lần seed 1, 2. Nhìn `top_share` = 0,99 của seed 0: 22 hit nhưng một mẫu duy nhất mang 99 % tổng w·y, tức "p = 4,6·10⁻³" thực ra là trọng số p/q của **một** kịch bản nằm ở đuôi mà q gần như không bao phủ, chia cho 600, không phải một tần suất. Seed 1 và 2 (top_share 0,20–0,26, khoảng ±1,96·se là [1,4·10⁻⁴, 6,4·10⁻⁴] và [5,5·10⁻⁵, 3,4·10⁻⁴]) **đồng ý với Monte Carlo** (`mc.json: p` = 6,7·10⁻⁴, Wilson [1,8·10⁻⁴, 2,4·10⁻³]) và hơi thấp hơn, đúng với cảnh báo của README (IS ở ngân sách này thường thấp hơn sự thật 1,5–4 lần vì q quá hẹp). `crosswalk_block` cũng có top_share > 0,5 ở cả ba seed: q của CEM co vào vùng đường ướt + người đi bộ chậm + không ngó qua xe buýt mà p(u) rất nhỏ, nên vài mẫu "gần tự nhiên" hơn mang hết trọng số; con số 10⁻⁷ vì vậy chỉ là bậc độ lớn, không phải ước lượng điểm đáng tin, nhưng vẫn nhất quán với Monte Carlo (0/3000, hi 1,28·10⁻³).

**Ngân sách.** `estimate.json: budget` = **16 800** lần chạy (4 kiểu × (8 vòng × 300 + 3 seed × 600)). Để Monte Carlo có cùng sai số chuẩn với seed 1 của `bus_bypass` (se = 1,27·10⁻⁴) cần n = p(1−p)/se² ≈ **2,4·10⁴** lần; với `crosswalk_block` (p ≈ 2,4–4,9·10⁻⁷, se ≈ 1,5–3,6·10⁻⁷) cần **3,6·10⁶ – 1,1·10⁷** lần (tính từ `p`, `se`). IS mua được bậc 10⁻⁷ bằng 16 800 lần chạy, Monte Carlo cần gấp hàng trăm lần.

**Phần `other`** lấy từ `mc.json: per_mode.other` (k = 0, n = 3000; `estimate.json: other.source` = mc.json) vì không có cuộc săn nào nhắm vào nó: "other" không phải một cơ chế mà là phần bù của bốn kiểu đã đặt tên, không có điểm nguy hiểm để CEM leo theo. Vì k = 0, tổng không cộng 0 mà cộng cận trên `other.hi` = 1,28·10⁻³ vào `seeds[].hi` (ví dụ seed 1: hi = 1,92·10⁻³).

## 5. Các kiểu thất bại (bước C2 + 3D)

`hunt.json: <mode>.found`: `bus_bypass` → {bus_bypass: **59**}; `cut_in` → {bus_bypass: 14}; `crosswalk_block` → {bus_bypass: 3, **crosswalk_block: 68**}; `rear_ended` → {bus_bypass: 2, crosswalk_block: 6}.

- **bus_bypass** — ảnh `evidence/shots/q5_bus_bypass_B-03.png` (B-03, 12,6 s). Chuỗi sự kiện: xe buýt đỗ ở làn E2 ngay trước vạch; C2 (xe tím) ở làn E2 phía sau xe buýt muốn lách sang E1 với khe chấp nhận rất nhỏ (`cutGap` 5–11 m trong các run B-01…B-04); C2 chậm (< 6 m/s, x < 112 m) lấn dần vào làn; planner chỉ coi C2 là xe phía trước khi |lat| < `cutinLat` = 2,15 m, lúc đó khoảng hở đã nhỏ hơn quãng phanh êm (b = 2,0) nên xe ta đâm vào đuôi/sườn C2.
- **crosswalk_block** — ảnh `evidence/shots/q5_crosswalk_block_C-01.png` (C-01, 20,4 s, đường ướt μ 0,5, người **không** ngó qua xe buýt). Chuỗi: xe buýt che tầm nhìn; người đi bộ P1 không ngó nên không thấy xe ta, bước xuống vạch; xe ta ở chế độ thận trọng (`cautionV` 7,5) nhưng đường ướt khiến quãng phanh dài; khi cảm biến thấy P1 thì phanh gấp và dừng được trước người, nhưng thân xe đã nằm trên vạch, chắn lối (`kind = block`).
- **Không tìm thấy:** `cut_in` và `rear_ended`. Mức tin: vừa phải. Cuộc săn `cut_in` bão hoà ở `trace[-1].level` = −3,45 (TTC tới C2 nhanh: elite đi −7,0 → −3,7 trong 4 vòng đầu rồi đứng ở −3,5 suốt 4 vòng sau), `rear_ended` ở −1,21 (TTC của TG ≥ 1,2 s): TG là IDM với b = 3,0 nên luôn kịp phanh theo xe ta, còn C2 nhanh (> 6 m/s) chỉ cắt khi có khe (`cutGap`) và đã ở xa. Nhưng chưa thấy ≠ không có: con số báo cho hai kiểu này là `estimate.json: upper_zero_hits` = 2,5·10⁻², và hai cuộc săn còn trôi sang kiểu khác (`cut_in` tìm ra 14 `bus_bypass`), cho thấy điểm nguy hiểm của chúng chưa phân biệt tốt.
- **Nếu chỉ chạy một cuộc săn chung:** CEM co về chỗ **dễ** nhất. Bằng chứng ngay trong `found`: cả cuộc săn `cut_in` lẫn `rear_ended` đều trượt sang `bus_bypass`/`crosswalk_block` dù điểm nguy hiểm của chúng khác. Một cuộc săn chung với điểm `min_ttc` sẽ cho ra ~70 va chạm `bus_bypass` (kiểu xảy ra ở TTC thấp nhất, sớm nhất, ~12,5 s) và không bao giờ tìm thấy `crosswalk_block` (xảy ra ở 20 s, cần đường ướt + không ngó), nên bảng ước lượng sẽ chỉ có một dòng.

## 6. Sửa planner (bước E + 3D)

`fix.json`:

| | policy | `fail` | `fail_hi` | `fail_hi_seen` | `progress` (m) | `harsh` |
|---|---|---|---|---|---|---|
| `baseline_train` (TRAIN) | gốc | 1,44·10⁻³ | 3,54·10⁻³ | 3,54·10⁻³ | 218,2 | 7,45 |
| `chosen` (TRAIN) | {cutinLat: **3,3**} | 2,69·10⁻⁶ | 6,98·10⁻⁶ | 6,98·10⁻⁶ | 217,8 | 7,28 |
| `chosen_test` (TEST) | {cutinLat: 3,3} | 5,58·10⁻⁸ | **3,88·10⁻²** | 1,05·10⁻⁷ | 216,8 | 7,58 |
| `baseline_test` (TEST) | gốc | 7,86·10⁻⁴ | 3,13·10⁻³ | 1,86·10⁻³ | 216,9 | 8,02 |

TRAIN chọn `cutinLat` = 3,3 vì nó có `fail_hi` thấp nhất trong các ứng viên giữ ≥ 95 % quãng đường (217,8 ≥ 0,95·218,2); trên TRAIN `bus_bypass` là `unseen` (0 hit/400). TEST (seed mới, săn lại nhắm vào chính policy này) cũng không tìm thấy `bus_bypass` nào (`chosen_test.bus_bypass.hits` = 0, `unseen` = [bus_bypass]), chỉ thấy `crosswalk_block` 5,6·10⁻⁸ (47 hit), và 0 va chạm nào khác trên 3 000 mẫu tự nhiên (`other_crashes.k` = 0). Tuy vậy `fail_hi` = 3,88·10⁻² **lớn hơn** `fail` = 7,9·10⁻⁴ của policy gốc: vì cận hợp lệ phải cộng `upper_zero_hits` = 3/(0,2·400) = 3,75·10⁻² cho kiểu đã "biến mất" và `other_crashes.hi` = 1,28·10⁻³ cho 0/3000. Bản sửa tốt hơn thật (fail 5,6·10⁻⁸ so với 7,9·10⁻⁴; `fail_hi_seen` 1,1·10⁻⁷ so với 1,9·10⁻³), nhưng với n = 400 không ngân sách nào ở đây *chứng nhận* `bus_bypass` xuống dưới 4·10⁻²; muốn chứng nhận < 10⁻³ cần cỡ 15 000 lần chạy nhắm đúng chỗ. Quãng đường gần như không đổi (216,8 so với 216,9 m) và phanh gấp ít hơn (7,58 so với 8,02 khung/kịch bản).

Ảnh có bóng mờ: `evidence/shots/q6_fix_ghost_B-01.png` (B-01, 12,6 s; run này ghim policy gốc nên xe đậm là policy gốc va chạm với C2, **bóng mờ là xe ta với `cutinLat` = 3,3**): bản sửa coi C2 là xe phía trước ngay khi C2 lấn 3,3 m, nên nhả ga sớm hơn ~1 s và còn cách C2 một thân xe lúc policy gốc đã đâm.

**Núm "nghe an toàn hơn" nhưng tăng thất bại:** `T` = 2,0 (giữ khoảng cách dài hơn, chạy chậm hơn) có `table[8]`: `crosswalk_block.p` = 2,95·10⁻⁶ với **102 hit**, nhiều nhất bảng, cao hơn policy gốc (2,69·10⁻⁶, 70 hit), trong khi quãng đường giảm còn 206,6 m. `cautionV` = 5,0 (bò chậm hơn gần vạch bị che) cùng chiều: `table[0]`: `crosswalk_block.p` = 3,71·10⁻⁶ > gốc (ước lượng này suy biến, `top_share` = 1,0 với 2 hit, nên chỉ là bậc độ lớn), nhưng dấu hiệu chắc hơn là phanh gấp tăng lên 11,8 khung/kịch bản (gốc 7,45) và quãng đường tụt còn 171,6 m: xe bò chậm vẫn phải phanh gấp nhiều hơn. Cơ chế là thế giới phản ứng: người đi bộ P1 quyết định bước xuống theo **thời gian tới** của xe gần nhất mà họ thấy (`tta` = khoảng cách/vận tốc so với `pedGap`); một xe ta bò 5 m/s trong 55 m trước vạch có tta lớn, người đi bộ chấp nhận khe và bước ra; xe ta tới vạch muộn hơn nhưng vẫn tới, và lúc đó người đang ở giữa đường, kết cục thân xe nằm trên vạch (block) hoặc phải phanh gấp liên tục. Chậm hơn không có nghĩa là an toàn hơn khi người khác lấy tốc độ của bạn để ra quyết định.

## 7. Từ mô phỏng ra đường thật

Ba giả định trong `avsim/space.py` ảnh hưởng nhiều nhất đến con số của tôi:

1. **Khe chấp nhận khi cắt làn `cutGap` ~ lognormal(median 14 m ở thị trấn D, σ 0,5)**: nó quyết định trực tiếp `bus_bypass` (các run B-01…B-04 có `cutGap` 5–11 m, tức đuôi trái của phân phối). Dữ liệu thay thế: phân phối khe chấp nhận khi nhập làn đo từ video/trajectory thật (NGSIM I-80/US-101, highD, hoặc video tại bến xe buýt nội đô Hà Nội có xe máy chen). Nếu median thật nhỏ hơn (lái xe Việt Nam chấp nhận khe ngắn hơn), xác suất `bus_bypass` của tôi **thấp hơn sự thật**; nếu lớn hơn, tôi báo cao.
2. **Tỉ lệ người đi bộ ngó qua xe buýt `pedLook` = 0,90 (Bernoulli)**: cả bốn run `crosswalk_block` đều có `pedLook` = không. Dữ liệu thay thế: quan sát hành vi nhìn trước khi qua đường tại vạch bị che (eye-tracking/video naturalistic; ví dụ các nghiên cứu "pedestrian looking behaviour at zebra crossings"), tách theo tuổi và việc dùng điện thoại. Nếu tỉ lệ không ngó thật là 25 % thay vì 10 %, `crosswalk_block` của tôi thấp hơn sự thật ~2,5 lần.
3. **Đường ướt `wet` = 0,15 và μ ướt = 0,5 cố định**: `crosswalk_block` chỉ tìm thấy trên đường ướt, và `is_feasible` cũng dựa vào μ. Dữ liệu thay thế: tần suất mưa theo giờ của thành phố (số liệu khí tượng; Hà Nội mùa mưa > 40 % số giờ chiều tối có đường ướt) và hệ số bám đo thật (μ ướt dải 0,35–0,6 tuỳ mặt đường, lốp). Nếu μ ướt thật thấp hơn 0,5 thì quãng phanh dài hơn: xác suất `crosswalk_block` của tôi **thấp hơn sự thật**, đồng thời một phần kịch bản tôi coi là khả thi (cutBrake, cutDur) lại bất khả thi.

Ngoài ra phân phối tự nhiên của p(u) giả định các toạ độ độc lập: ngoài đời trời mưa đi kèm người đi bộ vội và lái xe chấp nhận khe khác, nên tương quan thật có thể dồn thêm xác suất vào đúng góc nguy hiểm.

## 8. Giới hạn

Ba điều xác suất tôi báo **không** nói:

1. **Chỉ một con phố, một cấu hình**: một xe buýt đúng một chỗ (x ≈ 100–112), một vạch ở 122 m, một xe cắt làn C2 từ cùng một vị trí tương đối. Con số không nói gì về nút giao có đèn, đường hai chiều không dải phân cách, hay hai xe buýt nối đuôi che tầm nhìn dài hơn.
2. **Mọi người lái khác là IDM + luật khe**; không có xe máy, không ai vượt đèn, không ai lấn làn ngược chiều, TG luôn phanh được với b = 3,0 nên `rear_ended` gần như không thể xảy ra trong thế giới này, điều đó không đúng ngoài đời (người lái mất tập trung).
3. **Cảm biến hoàn hảo trừ phần bị xe buýt che**: không có trễ nhận thức, không mất phát hiện người đi bộ trong mưa, không nhiễu tracking; xác suất tôi báo là xác suất của *planner* với nhận thức lý tưởng, không phải của *hệ thống*. Thêm nữa, mô phỏng dừng ở 24 s và người đi bộ "không bao giờ bước vào thân xe", nên một số kết cục ngoài đời (người ngã, va chạm sau 24 s) bị cắt.

Thêm một tuần, tôi sẽ đo: (a) độ nhạy của `bus_bypass` theo `cut_gap_median` và `cutinLat` trên lưới 2D với IS cố định q để biết núm nào đáng chỉnh trước; (b) chạy 15 000 mẫu IS nhắm vào `bus_bypass` dưới policy đã chọn để có cận trên < 10⁻³ thay vì 3,9·10⁻²; (c) thêm trễ nhận thức 0,3–0,5 s và tỉ lệ bỏ sót phát hiện người đi bộ để xem `crosswalk_block` và rắc rối ở câu 9 tăng bao nhiêu.

## 9. Rắc rối của bạn (bước G + chương 10)

- **Rắc rối:** `hazard.json`: một người **băng ngang không có vạch** từ vỉa hè phía Nam, ngay phía trước đầu xe buýt đang đỗ ở bến (H0: `type` = ped, `side` = S, `x` = 92 m, `v` = 1,8 m/s, `trig` = 38 m, tức bắt đầu bước xuống khi xe ta còn cách 38 m). Ngoài đời đây là hành khách vừa xuống xe buýt hoặc chạy sang đường để kịp xe ở bến đối diện, băng qua giữa khối ngay trước mũi xe buýt.
- **Căn cứ vì sao planner kém:** đọc `avsim/core.py`, planner chỉ dự đoán ý định của **P1 tại vạch 122 m** (`will`, `pedBuffer`) và chỉ giảm tốc thận trọng trong `cautionRange` quanh *vạch*. Người đi bộ khác được coi như một "vật cản" chỉ khi đã nằm trong hành lang làn (|lat| ≤ 1,9 m). Nghĩa là với người băng ngang không có vạch, planner chỉ phản ứng khi người đã ở trong làn, lúc đó khoảng hở nhỏ hơn quãng phanh ở 10–12 m/s, dù cảm biến đã thấy người bước xuống vài giây trước.
- **Giả định** (`hazard.json: why`): `space` = {h0_x: [90, 104], h0_v: [1,0, 2,4], h0_trig: [18, 54]}, `exposure` = 0,04. `h0_x`: dải vỉa hè 90–100 m ngay trước đuôi xe buýt (xe buýt chiếm x ≈ 100–112, hướng +x), nơi hành khách xuống xe băng qua; phần 100–104 là người đứng ngang thân xe buýt, bị thân xe chặn (người đi bộ không bước vào thân xe) nên là phiên bản vô hại, giữ lại để `space` không chỉ gồm toàn kịch bản nguy hiểm; tránh vùng vạch 122 để không trùng kiểu `crosswalk_block` có sẵn. `h0_v`: 15th percentile đi bộ ≈ 1,0 m/s (Knoblauch 0,93–1,30; Exnicios 1,07; tổng hợp TRL PPR700), trung bình 1,2–1,56 m/s, đi vội trong pha clearance 1,56 m/s (Gates 2006), tối đa ≈ 1,3× tốc độ thường ≈ 1,9–2,4 m/s (Romero-Ortuno 2010); dưới ngưỡng chạy 2,5 m/s của `is_feasible`. `h0_trig`: khoảng cách tới xe đang đến mà người băng ngang giữa khối chấp nhận: mean 30,1 m, SD 12,1 m, mean ± 2SD ≈ 6–54 m, khe trung bình 3,3 s (Yannis, Papadimitriou & Theofilatos 2013, *Transportation Planning and Technology* 36(5), N = 243, Athens); cận dưới 18 m vì ở 10–12 m/s đó là < 1,5 s, gần như không ai chấp nhận. `exposure`: video 1 592 lượt xe buýt dừng ghi được 60 xung đột người đi bộ–xe = 0,038 xung đột mỗi lượt dừng (Soltani Mandolakani, Subedi, Singleton & Mekker 2024, arXiv:2408.08890, Utah); thị trấn này có đúng một bến với xe buýt đang đỗ mỗi chuyến → ≈ 0,04/chuyến. Số Mỹ, nơi có vạch gần đó; ở Hà Nội băng ngang giữa khối phổ biến hơn, nên đây có thể là cận dưới.
- **Kết quả** (`mine.json`, `exact.planner` = [ego|H0, 11,9 s] — cảnh dựng chạy lại đúng; `accepted` = **0,994** (1491/1500, `infeasible` = 9), `world` = town):

| policy | `crashes` | `avoidable` | `unavoidable` | `unattributed` | `low_speed` | `avoidable_foreknown` | `p` = P(lỗi planner \| rắc rối) [`lo`, `hi`] | `per_trip` (× 0,04) |
|---|---|---|---|---|---|---|---|---|
| `baseline` | 45 | **44** | 0 | 1 | 3 | 44 | **2,95·10⁻²** [2,21·10⁻², 3,94·10⁻²] | 1,18·10⁻³ (hi 1,58·10⁻³) |
| `chosen` {cutinLat: 3,3} | 46 | **46** | 0 | 0 | 3 | 46 | 3,09·10⁻² [2,32·10⁻², 4,09·10⁻²] | 1,23·10⁻³ (hi 1,64·10⁻³) |

  Mọi va chạm với H0 đều **avoidable**: phanh hết cỡ từ khung đầu tiên thấy người bước xuống là tránh được; `avoidable_foreknown` = `avoidable` vì người này không bị che (xe buýt nằm phía sau điểm băng ngang theo hướng nhìn của xe ta), nên cái giá của tầm nhìn bị che bằng 0 ở đây. `unavoidable` = 0: không phiên bản nào là "100 % va chạm không ai tránh nổi". 1 `unattributed` của policy gốc là một va chạm thường của thị trấn xảy ra khi H0 chưa bước xuống (`by_hazard.unattributed` = 1), không nói gì về rắc rối này. 3 `low_speed` mỗi policy: xe ta đã gần dừng (< 1 m/s) rồi bò chạm vào người, vẫn là lỗi planner (phanh quá muộn). Bản sửa ở bước E **không cứu** (46 so với 44, khoảng tin cậy chồng nhau): `cutinLat` chỉ đổi cách coi *xe* lấn làn, không chạm tới người đi bộ.
  Tần suất mỗi chuyến: 1,18·10⁻³ (cận trên 1,58·10⁻³), **gấp ~3 lần** `bus_bypass` của policy gốc theo IS (trung vị `estimate.json` 3,9·10⁻⁴; Monte Carlo 6,7·10⁻⁴) — ngay cả khi `exposure` thật chỉ bằng một phần tư số tôi dùng, rắc rối này vẫn cùng bậc với kiểu thất bại tệ nhất đã biết.
- **Hình:** `evidence/shots/q9_hazard_GB-02.png` (GB-02, chương 9, 12,3 s: xe ta đâm người băng ngang ngay trước đầu xe buýt, C2 phía sau) và `q9_hazard_chosen_GC-02.png` (cùng kịch bản với policy đã chọn: vẫn va chạm 12,3 s).
- **Bạn tự lái thì sao:** `hazard.json: ego` = script, `egoAcc` là pedal của tôi: giữ ga của planner đến lúc người bước xuống kerb (8,45 s), phản xạ 0,8 s rồi phanh 6 m/s² (dưới μg = 7,8), dừng ở x ≈ 71 m lúc 10,95 s, còn cách người 21 m, chờ người qua hết làn (13,85 s) rồi đi tiếp. `mine.json: exact` = {planner: [ego|H0, 11,9 s], **you: []**}: tôi tránh được, planner thì không. Khác biệt duy nhất là *lúc* bắt đầu phanh: tôi coi "người đang bước xuống lòng đường cách 38 m" là lý do phanh, planner đợi đến khi người đã ở trong làn. Oracle của `mine` nói điều tương tự ở quy mô 1 491 phiên bản: phanh hết cỡ từ lúc thấy (`examples[0]: fire_t` = `brake_t` = 8,8 s, va chạm của planner ở 12,6 s) tránh được 44/44 lần. Công bằng mà nói tôi đã biết trước cảnh này, còn planner thì không; đó cũng là lý do "tránh được" là chuẩn lạc quan ở dưới.
- **"Tránh được" là chuẩn lạc quan theo hai hướng:** oracle biết đúng khung hình rắc rối bắt đầu và phanh μ·g tức thì, không trễ nhận thức, không do dự, nên `avoidable` **đếm nhiều hơn** số vụ một planner thật (có trễ, phải phân biệt người sắp qua với người đứng chờ) tránh được: 2,95·10⁻² là cận trên của lỗi planner theo nghĩa đó. Ngược lại oracle **chỉ phanh**, không đánh lái sang E2 (trống vì xe buýt đỗ xa hơn), nên nó đếm **ít đi** những vụ có thể tránh bằng lách; con số thật nằm giữa.
- **Cách sửa:** không núm nào trong 9 núm sửa được: `pedBuffer`/`cautionV`/`cautionRange` chỉ gắn với P1 và toạ độ vạch 122 m; `T`, `s0`, `v0` giảm tốc chung cả chuyến (mất quãng đường) mà vẫn chỉ phản ứng khi người đã trong làn. Planner cần **logic mới**: (1) dự đoán ý định cho *mọi* người đi bộ, không chỉ P1 ở vạch — với mỗi người đang di chuyển có vector vận tốc hướng vào lòng đường, tính thời gian họ vào hành lang làn (t_enter = khoảng cách ngang/v_ngang) và thời gian xe ta tới (t_arrive); nếu t_enter < t_arrive + buffer thì coi họ là mục tiêu nhường như P1 (`mode` = 3); (2) vùng thận trọng quanh **xe buýt đang đỗ ở bến** (tương tự `cautionRange` nhưng neo vào xe buýt, không vào vạch), vì bến xe buýt là nơi người băng ngang xuất hiện. Cả hai không cần thêm cảm biến, chỉ dùng vị trí/vận tốc đã có trong `R.F`.
