MSSV: 202602571

# Lab 11 · Scenario Forge — báo cáo

Nguyễn Trần Kiên · MSSV trên lớp 2A202602571 (dòng đầu ghi dạng số để `run_all.py` đọc được).
Thị trấn (`meta.json: _run.town`): **D** (`p_bin` = wet 0,15 · pedLook 0,90 · brakes 0,20; `cut_gap_median` = 14 m). Seed `meta.json: seeds` = {mc 3354474268, search 3699743555, test 4018214513}; mọi bước `quick = false`. Số trích theo dạng `file.json: trường`; số suy ra từ chúng ghi "tính từ".

## 1. Phát lại log có trung thực không? (bước A)

| `replay.json` | giá trị |
|---|---|
| `n` / `dropped_human_crash_logs` / `n_used` | 400 / 0 / 400 |
| `replay.crashes` theo `contact` | **108** = `rear` 100 + `side` 8 |
| `reactive.crashes` | **0** |
| `crash_in_both` | 0 |

Cùng 400 kịch bản, planner trong thế giới phản ứng không va chạm; trên log phát lại có 108 va chạm (27 %), 100 là `rear` (TG đâm đuôi xe ta). Nguyên nhân: planner phanh sớm hơn và dừng hẳn ở chỗ người lái thật chỉ chậm lại — khi xe buýt che vạch, planner vào chế độ thận trọng (`cautionV` 7,5 m/s trong 55 m) rồi nhường người đi bộ (`pedBuffer`), còn người lái (HUMAN `T` 1,1 s, `b` 3,5) đi ~10 m/s qua vạch trước khi người bước xuống. TG trong log được ghi khi bám theo người lái thật; phát lại, TG lặp đúng gia tốc đã ghi và không biết phía trước là một planner đang phanh sớm hơn. Đó là va chạm **giả**: ngoài đời TG là người, thấy xe trước chậm thì phanh theo — thế giới phản ứng (IDM) mô phỏng đúng điều đó và cho 0 va chạm. `world.json` cho cùng kết luận: `replay_events` = [ego|TG, 14,8 s], `reactive_events` = [].

Kịch bản `replay.json: replay_only[0]` (`scenario` 4, `contact` rear, `min_ttc_reactive` 2,85 s; `tgT` 1,57 s, `pedT0` 8,1 s, `pedGap` 3,55 s, `pedLook` 1), chạy lại trong trình xem (R-01): từ ~9,5 s xe buýt che vạch, planner giảm còn ~7,2 m/s (mode 2); 15 s thấy P1 bước xuống, dừng ở x ≈ 114 m (mode 3). Người lái thật đi 9–10 m/s, qua vạch lúc ~14,7 s trước khi P1 xuống đường. TG phát lại chỉ giảm tốc theo người lái (10 → 4 m/s), tông xe ta đang đứng lúc 17,8 s. Trong thế giới phản ứng TTC nhỏ nhất của xe ta là 2,85 s. Ảnh: `evidence/shots/q1_replay_only_R-01_rear.png`.

## 2. Kịch bản bất khả thi (bước B)

`is_feasible` loại kịch bản nếu vi phạm bất kỳ luật nào:

| Luật | Căn cứ vật lý |
|---|---|
| (π/`cutDur`)²·3,5/2 > μ·g | Đổi làn 3,5 m theo quỹ đạo cosin có gia tốc ngang đỉnh (π/T)²·A/2; lốp chỉ bám μ·g (khô 7,8 m/s², ướt 4,9 m/s²). Đổi làn trong 1,2 s cần 12 m/s²: xe trượt. |
| `cutBrake` > μ·g | Giảm tốc dọc cũng bị giới hạn bởi ma sát lốp–đường. |
| `pedV` > 2,5 m/s | Trên 2,5 m/s là chạy; tốc độ đi bộ 15th–85th percentile nằm trong 0,9–1,6 m/s (TRL PPR700). |
| `cutV` ∉ (0, 22] m/s | Xe đứng yên/lùi không "cắt làn"; trên 22 m/s (79 km/h) vượt giới hạn phố nội đô của thế giới này. |

`hunt.json: <mode>.infeasible`: `crosswalk_block` **11** (nhiều nhất), `bus_bypass` 9, `rear_ended` 7, `cut_in` 2 (`mc.json: infeasible` = 12/3000). Điểm nguy hiểm của `crosswalk_block` là `ped_room` = khoảng cách tới người − v²/(2μg), nên đường ướt (quãng phanh ×1,6) được thưởng: CEM dồn về `wet` = 1 (`crosswalk_block.proposal.p_bin[0]` = 0,88 so với 0,15 tự nhiên) và về người đi bộ chậm (`proposal.mean[5]` = −2,40). Với μ = 0,5, luật gia tốc ngang đòi `cutDur` > 1,88 s thay vì 1,48 s, trong khi q vẫn để `cutDur` gần tự nhiên (`proposal.mean[1]` ≈ 0), nên đuôi ngắn của `cutDur` rơi vào vùng bất khả thi nhiều hơn các cuộc săn khác (lấy mẫu lại từ proposal cuối: toàn bộ mẫu bị loại là do luật gia tốc ngang). Nếu đếm chúng là va chạm, ước lượng IS bị thổi lên bởi kịch bản không ai tạo ra được (w·y > 0 ở nơi thế giới thật không có), CEM co nhanh hơn vào vùng vô lý đó và các kiểu thật như `bus_bypass` bị bỏ sót.

## 3. Monte Carlo cho bạn biết gì (bước C1)

`mc.json`: `n` = 3000, `crashes` = 2, `p` = **6,67·10⁻⁴**, Wilson 95 % [`lo`, `hi`] = **[1,83·10⁻⁴, 2,43·10⁻³]**, `modes` = {bus_bypass: 2}.

Để nửa khoảng còn ±20 % của p: 1,96·√(p(1−p)/n) = 0,2p ⇒ n ≈ 96/p ≈ **1,44·10⁵** lần chạy (tính từ `mc.json: p`), gấp 48 lần ngân sách hiện có (~96 va chạm quan sát).

Kiểu chưa xuất hiện trong 3 000 lần: `cut_in`, `crosswalk_block`, `rear_ended`, `other` (`per_mode.*.k` = 0). Con số cần báo không phải 0 mà là cận trên Wilson với k = 0: `per_mode.<kiểu>.hi` = **1,28·10⁻³** (lo = 0) — Monte Carlo chỉ nói "hiếm hơn 1,3‰", không nói "không có".

## 4. Importance sampling, ESS và độ suy biến (bước C3)

`estimate.json`, n = 600 mẫu/kiểu/seed, q = 0,8·CEM + 0,2·tự nhiên:

| kiểu | seed | `p` | `hits` | `ess` | `top_share` |
|---|---|---|---|---|---|
| bus_bypass | 0 | 4,64·10⁻³ | 22 | 129 | **0,99** |
| bus_bypass | 1 | 3,87·10⁻⁴ | 31 | 124 | 0,20 |
| bus_bypass | 2 | 1,98·10⁻⁴ | 22 | 138 | 0,26 |
| cut_in | 0/1/2 | 0 | 0 | 124 / 108 / 128 | — (`upper_zero_hits` = 2,5·10⁻²) |
| crosswalk_block | 0 | 4,91·10⁻⁷ | 97 | 123 | **0,55** |
| crosswalk_block | 1 | 2,38·10⁻⁷ | 94 | 118 | **0,61** |
| crosswalk_block | 2 | 4,65·10⁻⁷ | 97 | 132 | **0,77** |
| rear_ended | 0/1/2 | 0 | 0 | 110 / 121 / 114 | — (`upper_zero_hits` = 2,5·10⁻²) |
| `other` | — | 0 (k = 0/3000) | | | `other.hi` = 1,28·10⁻³ |
| **total** | 0 / 1 / 2 | 4,64·10⁻³ / 3,88·10⁻⁴ / 1,99·10⁻⁴ | | | trung vị **3,88·10⁻⁴** (tính từ `seeds[].total`) |

**Đồng thuận.** `crosswalk_block` nhất quán giữa ba seed (2–5·10⁻⁷). `bus_bypass` không: seed 0 cao gấp 12–23 lần với `top_share` 0,99 — một mẫu duy nhất mang 99 % tổng w·y, nên "4,6·10⁻³" là trọng số p/q của một kịch bản ở đuôi mà q gần như không phủ, chia cho 600, không phải tần suất. Seed 1, 2 (top_share 0,20–0,26; ±1,96·se: [1,4·10⁻⁴, 6,4·10⁻⁴] và [5,5·10⁻⁵, 3,4·10⁻⁴]) nhất quán với Monte Carlo (`mc.json: p` 6,7·10⁻⁴, Wilson [1,8·10⁻⁴, 2,4·10⁻³]) và hơi thấp hơn, đúng với độ chệch 1,5–4 lần README nêu. `crosswalk_block` có top_share > 0,5 ở cả ba seed vì q co vào vùng ướt + người chậm + không ngó, nơi p(u) rất nhỏ; vài mẫu "gần tự nhiên" mang hết trọng số, nên 10⁻⁷ là bậc độ lớn, không phải ước lượng điểm, song nhất quán với Monte Carlo (0/3000, hi 1,28·10⁻³).

**Ngân sách.** `estimate.json: budget` = 16 800 lần chạy. Monte Carlo đạt cùng se với seed 1 của `bus_bypass` (se 1,27·10⁻⁴) cần n = p(1−p)/se² ≈ **2,4·10⁴**; với `crosswalk_block` (p 2,4–4,9·10⁻⁷, se 1,5–3,6·10⁻⁷) cần **3,6·10⁶ – 1,1·10⁷** (tính từ `p`, `se`): IS mua bậc 10⁻⁷ với giá rẻ hơn hàng trăm lần.

**`other`** lấy từ `mc.json: per_mode.other` (k = 0, n = 3000; `estimate.json: other.source` = mc.json) vì không săn được: nó là phần bù của bốn kiểu đã đặt tên, không có điểm nguy hiểm riêng để CEM leo. Vì k = 0, tổng cộng cận trên `other.hi` = 1,28·10⁻³ vào `seeds[].hi` (seed 1: hi = 1,92·10⁻³).

## 5. Các kiểu thất bại (bước C2 + 3D)

`hunt.json: <mode>.found`: `bus_bypass` → {bus_bypass: **59**}; `cut_in` → {bus_bypass: 14}; `crosswalk_block` → {bus_bypass: 3, **crosswalk_block: 68**}; `rear_ended` → {bus_bypass: 2, crosswalk_block: 6}.

- **bus_bypass** — `evidence/shots/q5_bus_bypass_B-03.png` (B-03, 12,6 s). Xe buýt đỗ ở E2 trước vạch; C2 phía sau xe buýt lách sang E1 với khe chấp nhận nhỏ (`cutGap` 5–11 m ở B-01…B-04), chậm (< 6 m/s, x < 112 m) và lấn dần; planner chỉ coi C2 là xe phía trước khi |lat| < `cutinLat` 2,15 m, lúc đó khoảng hở đã nhỏ hơn quãng phanh êm (b = 2,0).
- **crosswalk_block** — `evidence/shots/q5_crosswalk_block_C-01.png` (C-01, 20,4 s, μ 0,5, người không ngó). Xe buýt che tầm nhìn; P1 không ngó nên bước xuống vạch; xe ta ở `cautionV` 7,5 nhưng đường ướt kéo dài quãng phanh; thấy P1 thì phanh gấp và dừng trước người, nhưng thân xe đã nằm trên vạch (`kind` = block).
- **Không tìm thấy:** `cut_in`, `rear_ended`. Cuộc săn `cut_in` bão hoà ở `trace[-1].level` = −3,45 (elite đi −7,0 → −3,7 trong 4 vòng rồi đứng ở −3,5), `rear_ended` ở −1,21: TG là IDM với b = 3,0 nên luôn kịp phanh; C2 nhanh (> 6 m/s) chỉ cắt khi có khe. Mức tin vừa phải: hai cuộc săn còn trôi sang kiểu khác (`cut_in` tìm ra 14 `bus_bypass`), và con số báo cáo cho hai kiểu này là `upper_zero_hits` = 2,5·10⁻², không phải 0.
- **Một cuộc săn chung** sẽ co về kiểu dễ nhất: ngay trong `found`, cả cuộc săn `cut_in` lẫn `rear_ended` đã trượt sang `bus_bypass`/`crosswalk_block`. Săn chung theo `min_ttc` sẽ cho ~70 `bus_bypass` (TTC thấp nhất, sớm nhất ~12,5 s) và không bao giờ tới `crosswalk_block` (20 s, cần ướt + không ngó); bảng ước lượng chỉ còn một dòng.

## 6. Sửa planner (bước E + 3D)

`fix.json`:

| | policy | `fail` | `fail_hi` | `fail_hi_seen` | `progress` (m) | `harsh` |
|---|---|---|---|---|---|---|
| `baseline_train` | gốc | 1,44·10⁻³ | 3,54·10⁻³ | 3,54·10⁻³ | 218,2 | 7,45 |
| `chosen` (TRAIN) | {cutinLat: **3,3**} | 2,69·10⁻⁶ | 6,98·10⁻⁶ | 6,98·10⁻⁶ | 217,8 | 7,28 |
| `chosen_test` (TEST) | {cutinLat: 3,3} | 5,58·10⁻⁸ | **3,88·10⁻²** | 1,05·10⁻⁷ | 216,8 | 7,58 |
| `baseline_test` (TEST) | gốc | 7,86·10⁻⁴ | 3,13·10⁻³ | 1,86·10⁻³ | 216,9 | 8,02 |

TRAIN chọn `cutinLat` 3,3: `fail_hi` thấp nhất trong các ứng viên giữ ≥ 95 % quãng đường (217,8 ≥ 0,95·218,2); `bus_bypass` là `unseen`. TEST (seed mới, săn lại nhắm vào chính policy) vẫn không tìm thấy `bus_bypass` (`chosen_test.bus_bypass.hits` = 0), chỉ `crosswalk_block` 5,6·10⁻⁸ (47 hit) và 0 va chạm khác trên 3 000 mẫu tự nhiên (`other_crashes.k` = 0). `fail_hi` = 3,88·10⁻² vẫn lớn hơn `fail` = 7,9·10⁻⁴ của policy gốc vì cận hợp lệ phải cộng `upper_zero_hits` = 3/(0,2·400) = 3,75·10⁻² cho kiểu đã biến mất và `other_crashes.hi` = 1,28·10⁻³ cho 0/3000. Bản sửa tốt hơn thật (`fail` 5,6·10⁻⁸ so với 7,9·10⁻⁴; `fail_hi_seen` 1,1·10⁻⁷ so với 1,9·10⁻³), nhưng n = 400 không chứng nhận được `bus_bypass` dưới 4·10⁻²; chứng nhận < 10⁻³ cần ~15 000 lần chạy nhắm đúng chỗ. Quãng đường không đổi (216,8 so với 216,9 m), phanh gấp ít hơn (7,58 so với 8,02).

Ảnh có bóng mờ: `evidence/shots/q6_fix_ghost_B-01.png` (B-01, 12,6 s; xe đậm là policy gốc va chạm với C2, bóng mờ là xe ta với `cutinLat` 3,3): bản sửa coi C2 là xe phía trước ngay khi C2 lấn 3,3 m, nhả ga sớm hơn ~1 s và còn cách C2 một thân xe lúc policy gốc đã đâm.

**Núm "an toàn hơn" phản tác dụng.** `T` = 2,0 (`table[8]`): `crosswalk_block.p` 2,95·10⁻⁶ với 102 hit — nhiều nhất bảng, cao hơn gốc (2,69·10⁻⁶, 70 hit) — và quãng đường còn 206,6 m. `cautionV` = 5,0 (`table[0]`) cùng chiều: `crosswalk_block.p` 3,71·10⁻⁶ (ước lượng suy biến, `top_share` 1,0 với 2 hit, chỉ là bậc độ lớn), `harsh` 11,8 (gốc 7,45), quãng đường 171,6 m. Cơ chế: P1 quyết định bước xuống theo thời gian tới của xe gần nhất nhìn thấy (`tta` so với `pedGap`); xe ta chạy chậm có `tta` lớn, người chấp nhận khe và bước ra; xe ta tới vạch muộn hơn nhưng vẫn tới khi người đang giữa đường, kết cục là chặn lối hoặc phanh gấp liên tục. Chậm hơn không an toàn hơn khi người khác lấy tốc độ của xe ta làm căn cứ quyết định.

## 7. Từ mô phỏng ra đường thật

| Giả định (`space.py`) | Dữ liệu thật thay thế | Nếu sai, con số lệch hướng nào |
|---|---|---|
| `cutGap` ~ lognormal(median 14 m, σ 0,5): quyết định `bus_bypass` (B-01…B-04 có `cutGap` 5–11 m, đuôi trái) | Phân phối khe chấp nhận khi nhập làn từ trajectory thật: NGSIM I-80/US-101, highD, hoặc video bến xe buýt nội đô Hà Nội có xe máy | Median thật nhỏ hơn (lái xe Việt Nam chấp nhận khe ngắn) → `bus_bypass` của tôi **thấp** hơn sự thật; lớn hơn → cao |
| `pedLook` = 0,90: cả bốn run `crosswalk_block` đều có `pedLook` = 0 | Quan sát hành vi nhìn trước khi qua vạch bị che (video naturalistic / eye-tracking), tách theo tuổi và dùng điện thoại | Tỉ lệ không ngó thật 25 % thay vì 10 % → `crosswalk_block` thấp hơn sự thật ~2,5 lần |
| `wet` = 0,15, μ ướt = 0,5 cố định: `crosswalk_block` chỉ tìm thấy trên đường ướt; `is_feasible` cũng dùng μ | Tần suất giờ mưa từ khí tượng (Hà Nội mùa mưa > 40 % giờ chiều tối đường ướt); μ ướt đo thật (0,35–0,6 tuỳ mặt đường, lốp) | μ ướt thật thấp hơn → quãng phanh dài hơn → `crosswalk_block` **thấp** hơn sự thật; một phần kịch bản tôi coi khả thi lại bất khả thi |

Ngoài ra p(u) giả định các toạ độ độc lập; ngoài đời mưa đi kèm người đi vội và khe chấp nhận khác, tương quan thật có thể dồn thêm xác suất vào đúng góc nguy hiểm.

## 8. Giới hạn

1. **Một con phố, một cấu hình**: xe buýt ở đúng x ≈ 100–112, một vạch ở 122 m, C2 luôn xuất phát cùng vị trí tương đối. Không nói gì về nút giao có đèn, đường hai chiều không dải phân cách, hay hai xe buýt nối đuôi.
2. **Mọi xe khác là IDM + luật khe**: không xe máy, không vượt đèn, không lấn làn ngược chiều; TG luôn phanh kịp với b = 3,0 nên `rear_ended` gần như bất khả thi trong thế giới này, khác ngoài đời.
3. **Cảm biến hoàn hảo trừ phần xe buýt che**: không trễ nhận thức, không mất phát hiện trong mưa, không nhiễu tracking; xác suất báo cáo là của planner với nhận thức lý tưởng, không phải của hệ thống. Mô phỏng dừng ở 24 s và người đi bộ không bước vào thân xe, nên một số kết cục bị cắt.

Thêm một tuần: (a) quét độ nhạy `bus_bypass` theo `cut_gap_median` × `cutinLat` với q cố định; (b) 15 000 mẫu IS nhắm `bus_bypass` dưới policy đã chọn để có cận trên < 10⁻³ thay vì 3,9·10⁻²; (c) thêm trễ nhận thức 0,3–0,5 s và tỉ lệ bỏ sót người đi bộ, đo `crosswalk_block` và rắc rối ở câu 9 tăng bao nhiêu.

## 9. Rắc rối của bạn (bước G + chương 10)

- **Rắc rối.** `hazard.json`: người **băng ngang không có vạch** từ vỉa hè Nam, ngay trước xe buýt đang đỗ ở bến (H0: `type` ped, `side` S, `x` 92 m, `v` 1,8 m/s, `trig` 38 m — bước xuống khi xe ta còn cách 38 m). Ngoài đời: hành khách vừa xuống xe buýt hoặc chạy sang đường để kịp xe, băng qua giữa khối.
- **Căn cứ.** Trong `avsim/core.py`, planner chỉ dự đoán ý định của P1 tại vạch 122 m (`will`, `pedBuffer`) và chỉ thận trọng trong `cautionRange` quanh vạch; người đi bộ khác chỉ được coi là vật cản khi đã ở trong hành lang làn (|lat| ≤ 1,9 m). Với người băng ngang không vạch, planner phản ứng khi người đã trong làn, lúc đó khoảng hở nhỏ hơn quãng phanh ở 10–12 m/s dù cảm biến đã thấy người từ vài giây trước.
- **Giả định** (`hazard.json: why`): `space` = {h0_x [90, 104], h0_v [1,0, 2,4], h0_trig [18, 54]}, `exposure` = 0,04.
  - `h0_x`: vỉa hè 90–100 m ngay trước đuôi xe buýt (xe chiếm 100–112, hướng +x), nơi hành khách xuống xe băng qua; 100–104 là người ngang thân xe, bị thân xe chặn nên vô hại — giữ lại để `space` không chỉ gồm kịch bản nguy hiểm; tránh vùng vạch 122 để không trùng `crosswalk_block`.
  - `h0_v`: 15th percentile đi bộ ≈ 1,0 m/s (Knoblauch 0,93–1,30; Exnicios 1,07; TRL PPR700), trung bình 1,2–1,56, đi vội 1,56 (Gates 2006), tối đa ≈ 1,3× ≈ 1,9–2,4 (Romero-Ortuno 2010); dưới ngưỡng chạy 2,5 m/s.
  - `h0_trig`: khoảng cách tới xe đang đến mà người băng ngang giữa khối chấp nhận: mean 30,1 m, SD 12,1 m (mean ± 2SD ≈ 6–54 m), khe 3,3 s (Yannis, Papadimitriou & Theofilatos 2013, *Transportation Planning and Technology* 36(5), N = 243); cận dưới 18 m vì ở 10–12 m/s đó là < 1,5 s.
  - `exposure`: 60 xung đột người đi bộ–xe trên 1 592 lượt xe buýt dừng = 0,038/lượt (Soltani Mandolakani, Subedi, Singleton & Mekker 2024, arXiv:2408.08890); thị trấn có một bến với xe buýt đang đỗ mỗi chuyến → 0,04/chuyến. Số liệu Mỹ, nơi có vạch gần đó; ở Hà Nội băng ngang phổ biến hơn, nên đây là cận dưới.
- **Kết quả** (`mine.json`): `exact.planner` = [ego|H0, 11,9 s]; `accepted` = **0,994** (1491/1500, `infeasible` 9); `world` = town.

| policy | `crashes` | `avoidable` | `unavoidable` | `unattributed` | `low_speed` | `avoidable_foreknown` | P(lỗi planner \| rắc rối) [`lo`, `hi`] | `per_trip` (× 0,04) |
|---|---|---|---|---|---|---|---|---|
| `baseline` | 45 | **44** | 0 | 1 | 3 | 44 | **2,95·10⁻²** [2,21·10⁻², 3,94·10⁻²] | 1,18·10⁻³ (hi 1,58·10⁻³) |
| `chosen` {cutinLat: 3,3} | 46 | **46** | 0 | 0 | 3 | 46 | 3,09·10⁻² [2,32·10⁻², 4,09·10⁻²] | 1,23·10⁻³ (hi 1,64·10⁻³) |

  Mọi va chạm với H0 đều `avoidable`; `avoidable_foreknown` = `avoidable` vì người này không bị che, nên cái giá của tầm nhìn bị che bằng 0. `unavoidable` = 0: không phiên bản nào "không ai tránh nổi". 1 `unattributed` của policy gốc là va chạm thường của thị trấn khi H0 chưa bước xuống (`by_hazard.unattributed` = 1). 3 `low_speed` mỗi policy: xe gần dừng rồi bò chạm người, vẫn là phanh quá muộn. Bản sửa ở bước E không cứu (46 so với 44, khoảng tin cậy chồng nhau): `cutinLat` chỉ đổi cách xử lý xe lấn làn. Tần suất 1,18·10⁻³/chuyến (cận trên 1,58·10⁻³) gấp ~3 lần `bus_bypass` của policy gốc (trung vị IS 3,9·10⁻⁴; Monte Carlo 6,7·10⁻⁴); kể cả khi `exposure` thật chỉ bằng một phần tư, rắc rối này vẫn cùng bậc với kiểu thất bại tệ nhất đã biết.
- **Hình.** `evidence/shots/q9_hazard_GB-02.png` (GB-02, 12,3 s: xe ta đâm người băng ngang trước xe buýt) và `q9_hazard_chosen_GC-02.png` (cùng kịch bản, policy đã chọn: vẫn va chạm 12,3 s).
- **Tự lái.** `hazard.json: ego` = script, `egoAcc` là pedal của tôi: giữ ga của planner tới khi người bước xuống (8,45 s), phản xạ 0,8 s rồi phanh 6 m/s² (dưới μg 7,8), dừng ở x ≈ 71 m lúc 10,95 s, cách người 21 m, chờ người qua làn (13,85 s) rồi đi tiếp. `mine.json: exact` = {planner: [ego|H0, 11,9 s], you: []}: tôi tránh được, planner không. Khác biệt duy nhất là thời điểm bắt đầu phanh: tôi coi "người đang bước xuống lòng đường cách 38 m" là lý do phanh, planner đợi người vào làn. Oracle của `mine` nói điều tương tự trên 1 491 phiên bản: phanh hết cỡ từ lúc thấy (`examples[0]: fire_t` = `brake_t` = 8,8 s; planner va 12,6 s) tránh 44/44. Tôi đã biết trước cảnh này, planner thì không — cũng là lý do "tránh được" là chuẩn lạc quan.
- **"Tránh được" lạc quan theo hai hướng.** Oracle biết đúng khung hình rắc rối bắt đầu và phanh μ·g tức thì, không trễ, không do dự, nên `avoidable` đếm **nhiều hơn** số vụ một planner thật tránh được: 2,95·10⁻² là cận trên của lỗi planner. Ngược lại oracle chỉ phanh, không đánh lái sang E2 đang trống, nên đếm **ít đi** những vụ tránh được bằng lách; con số thật nằm giữa.
- **Cách sửa.** Không núm nào sửa được: `pedBuffer`/`cautionV`/`cautionRange` chỉ gắn với P1 và toạ độ vạch; `T`, `s0`, `v0` giảm tốc cả chuyến mà vẫn chỉ phản ứng khi người đã trong làn. Cần logic mới: (1) dự đoán ý định cho mọi người đi bộ — với người đang di chuyển có vận tốc hướng vào lòng đường, tính t_enter = khoảng cách ngang / v_ngang và t_arrive của xe ta; nếu t_enter < t_arrive + buffer thì nhường như P1 (`mode` 3); (2) vùng thận trọng neo vào xe buýt đang đỗ ở bến thay vì vào vạch. Cả hai chỉ dùng vị trí/vận tốc đã có trong `R.F`, không cần thêm cảm biến.
