// Unit test tầng business — thuần logic, KHÔNG cần GPU/DeepStream.
// Link với vehicle_business (rules/track/plate_recognizer), chạy < 1s.
//
// Chạy:  ./build/vehicle_business_tests
// Mọi case in ra PASS/FAIL; exit code != 0 nếu có case fail.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "business/plate/plate_recognizer.h"
#include "business/plate/rules.h"
#include "business/plate/track.h"

namespace {

int g_failed = 0;
int g_total = 0;

void check(bool ok, const char* what) {
  ++g_total;
  if (ok) {
    std::printf("  [PASS] %s\n", what);
    return;
  }
  ++g_failed;
  std::printf("  [FAIL] %s\n", what);
}

void checkEq(const std::string& got, const std::string& want, const char* what) {
  const bool ok = got == want;
  ++g_total;
  if (ok) {
    std::printf("  [PASS] %s\n", what);
    return;
  }
  ++g_failed;
  std::printf("  [FAIL] %s — got '%s', want '%s'\n", what, got.c_str(), want.c_str());
}

void checkEqInt(long long got, long long want, const char* what) {
  const bool ok = got == want;
  ++g_total;
  if (ok) {
    std::printf("  [PASS] %s\n", what);
    return;
  }
  ++g_failed;
  std::printf("  [FAIL] %s — got %lld, want %lld\n", what, got, want);
}

using vehicle::business::plate::CharSequence;
using vehicle::business::plate::DedupCache;
using vehicle::business::plate::PlateOcrStatus;
using vehicle::business::plate::TrackPlateState;
using vehicle::CharReading;

// Dựng CharSequence từ chuỗi, mọi ký tự cùng confidence.
CharSequence makeChars(const std::string& text, double conf = 0.9) {
  CharSequence out;
  out.reserve(text.size());
  for (char c : text) out.push_back(CharReading{std::string(1, c), conf});
  return out;
}

// ---------------------------------------------------------------------------
// rules
// ---------------------------------------------------------------------------
void testRules() {
  std::printf("== rules ==\n");
  namespace p = vehicle::business::plate;

  check(p::isPlateValid("14A12345"), "isPlateValid('14A12345')");
  check(!p::isPlateValid(""), "isPlateValid('') là false");
  check(!p::isPlateValid("???"), "isPlateValid('???') là false");

  checkEq(p::normalizePlateForEmit(""), p::kUnknownPlate, "normalize('') → UNKOWN");
  checkEq(p::normalizePlateForEmit(p::kEmptyPlate), p::kUnknownPlate,
          "normalize('EMPTY') → UNKOWN");
  checkEq(p::normalizePlateForEmit("14A12345"), "14A12345", "normalize biển hợp lệ giữ nguyên");

  // send_mode 1 = chỉ biển tốt; 2 = bắn tất cả; khác = không bắn.
  check(p::shouldEmitPlate("14A12345", 1), "send_mode=1 bắn biển hợp lệ");
  check(!p::shouldEmitPlate(p::kUnknownPlate, 1), "send_mode=1 KHÔNG bắn UNKOWN");
  check(p::shouldEmitPlate(p::kUnknownPlate, 2), "send_mode=2 bắn cả UNKOWN");
  check(!p::shouldEmitPlate("14A12345", 0), "send_mode=0 không bắn gì");
}

// ---------------------------------------------------------------------------
// DedupCache
//
// Case "hai xe khác nhau cùng chuỗi biển" HIỆN ĐANG FAIL — đó là chủ đích.
// DedupCache hiện là 2 deque khớp OR (track_id HOẶC plate), nên xe thứ hai bị
// nuốt. Test này là bằng chứng của bug; nó chuyển xanh ở commit sửa DedupCache.
// ---------------------------------------------------------------------------
void testDedupCache() {
  std::printf("== DedupCache ==\n");

  using vehicle::business::plate::EventKind;
  const EventKind kP = EventKind::kPlate;

  {
    DedupCache cache;
    check(cache.tryEmit(1, "14A12345", kP), "lần đầu (track 1) được bắn");
    check(!cache.tryEmit(1, "14A12345", kP), "cùng track + cùng biển + cùng kind bị chặn");
  }

  {
    // Hai TRACK KHÁC NHAU đọc ra cùng chuỗi biển — hai xe khác nhau, cả hai
    // đều phải được bắn. Rất phổ biến với 'UNKOWN' (biển không đọc được).
    DedupCache cache;
    check(cache.tryEmit(1, vehicle::business::plate::kUnknownPlate, kP),
          "track 1 UNKOWN được bắn");
    check(cache.tryEmit(2, vehicle::business::plate::kUnknownPlate, kP),
          "track 2 UNKOWN cũng phải được bắn (xe khác)");
    check(cache.tryEmit(3, "14A12345", kP), "track 3 biển khác được bắn");
    check(cache.tryEmit(4, "14A12345", kP), "track 4 cùng biển với track 3 vẫn phải bắn");
  }

  {
    // Cùng track + cùng biển nhưng KIND khác nhau → mỗi nghiệp vụ bắn riêng.
    DedupCache cache;
    check(cache.tryEmit(5, "14A12345", EventKind::kPlate), "track 5: PLATE");
    check(cache.tryEmit(5, "14A12345", EventKind::kWrongLane),
          "track 5: WRONG_LANE không bị PLATE chặn");
    check(cache.tryEmit(5, "14A12345", EventKind::kNoHelmet),
          "track 5: NO_HELMET không bị chặn");
    check(!cache.tryEmit(5, "14A12345", EventKind::kWrongLane),
          "track 5: WRONG_LANE lần hai bị chặn");
  }

  {
    // forget: publish fail → gỡ khoá để retry được.
    DedupCache cache;
    check(cache.tryEmit(6, "14A12345", kP), "track 6: bắn lần đầu");
    check(!cache.tryEmit(6, "14A12345", kP), "track 6: bị chặn khi chưa forget");
    cache.forget(6, "14A12345", kP);
    check(cache.tryEmit(6, "14A12345", kP), "track 6: sau forget thì retry được");
  }

  {
    // TTL: entry hết hạn thì bắn lại được.
    DedupCache cache(50, /*ttl_s=*/10.0);
    check(cache.tryEmit(7, "14A12345", kP, /*now_s=*/100.0), "TTL: bắn lúc t=100");
    check(!cache.tryEmit(7, "14A12345", kP, 105.0), "TTL: t=105 vẫn trong hạn → chặn");
    check(cache.tryEmit(7, "14A12345", kP, 111.0), "TTL: t=111 quá hạn → bắn lại được");
  }

  {
    // maxlen: entry cũ nhất bị đẩy ra, track cũ được bắn lại.
    DedupCache cache(2);
    check(cache.tryEmit(10, "A", kP), "cache(2): track 10");
    check(cache.tryEmit(11, "B", kP), "cache(2): track 11");
    check(cache.tryEmit(12, "C", kP), "cache(2): track 12 đẩy track 10 ra");
    check(cache.tryEmit(10, "A", kP), "cache(2): track 10 bắn lại được sau khi bị đẩy ra");
  }
}

// ---------------------------------------------------------------------------
// TrackPlateState
// ---------------------------------------------------------------------------
void testTrackPlateState() {
  std::printf("== TrackPlateState ==\n");

  {
    // Chưa vào zone thì không nhận OCR.
    TrackPlateState st(1, 0.0, 3);
    check(!st.canOcr(), "ngoài zone: canOcr() false");
    check(st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0) ==
              PlateOcrStatus::kRejected,
          "ngoài zone: addOcrReading bị từ chối");
  }

  {
    // Đủ max_recognize_times → chốt biển. area không tăng nên không bị hoãn.
    TrackPlateState st(2, 0.0, 3);
    st.onEnterPolygon(0.0);
    PlateOcrStatus last = PlateOcrStatus::kRejected;
    for (int i = 0; i < 3; ++i) {
      last = st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0 + i, 0.9, 100.0, 40.0);
    }
    check(vehicle::business::plate::plateOcrJustFinal(last), "đủ N reading → kFinalized");
    check(st.hasFinalPlate(), "hasFinalPlate() sau khi chốt");
    checkEq(st.plate(), "14A12345", "chuỗi biển chốt đúng");
    checkEq(st.bestSnapshotKey(), "14A12345", "bestSnapshotKey khớp biển đã chốt");
    checkEqInt(st.plateRecognizeCount(), 3, "plateRecognizeCount");
  }

  {
    // Vote: chuỗi được nhiều phiếu hơn thắng, không phải chuỗi conf cao hơn.
    TrackPlateState st(3, 0.0, 4);
    st.onEnterPolygon(0.0);
    st.addOcrReading(makeChars("14A99999", 0.99), "14A99999", 1.0, 0.99, 100.0, 40.0);
    for (int i = 0; i < 3; ++i) {
      st.addOcrReading(makeChars("14A12345", 0.80), "14A12345", 2.0 + i, 0.80, 100.0, 40.0);
    }
    check(st.hasFinalPlate(), "vote: đã chốt");
    checkEq(st.plate(), "14A12345", "vote: chuỗi nhiều phiếu thắng dù conf thấp hơn");
  }

  {
    // Vote class: nhiều phiếu nhất thắng.
    TrackPlateState st(4, 0.0);
    checkEqInt(st.votedCls(), -1, "votedCls() = -1 khi chưa có phiếu");
    st.addClass(2, 0.9);
    st.addClass(2, 0.8);
    st.addClass(0, 0.95);
    checkEqInt(st.votedCls(), 2, "votedCls() lấy class nhiều phiếu nhất");
  }

  {
    // Miss-finalize: rời zone quá kMissTrackIdleS → chốt bằng số reading đang có.
    TrackPlateState st(5, 0.0, 20);
    st.onEnterPolygon(0.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0, 0.9, 100.0, 40.0);
    check(!st.hasFinalPlate(), "chưa đủ N reading thì chưa chốt");
    st.onLeavePolygon(2.0);
    check(!st.tryFinalizeMiss(2.5), "rời zone chưa quá 1s: chưa miss-finalize");
    check(st.tryFinalizeMiss(4.0), "rời zone quá 1s: miss-finalize");
    check(st.hasFinalPlate(), "miss-finalize đã chốt biển");
  }

  {
    // Xoá track: quá tuổi tối đa.
    TrackPlateState st(6, 0.0, 20);
    check(!st.shouldForceDelete(1.0), "track mới không bị xoá");
    check(st.shouldForceDelete(vehicle::business::plate::kForceDeleteAgeS + 1.0),
          "quá kForceDeleteAgeS thì xoá");
  }
}

// ---------------------------------------------------------------------------
// TrackPlateState — vòng đời per-kind
// ---------------------------------------------------------------------------
void testKindLifecycle() {
  std::printf("== vòng đời per-kind ==\n");
  using vehicle::business::plate::EventKind;
  namespace p = vehicle::business::plate;

  {
    TrackPlateState st(20, 0.0);
    checkEqInt(st.activeKinds(), 0, "track mới: không kind nào active");
    check(st.allSettled(), "track mới: allSettled() true (chưa có gì để bắn)");

    st.addWrongWayObservation("REVERSE_DIRECTION", 5.0);
    checkEqInt(st.kind(EventKind::kWrongWay).hits, 1, "WRONG_WAY: hits = 1");
    checkEq(st.kindLabel(EventKind::kWrongWay), "REVERSE_DIRECTION", "WRONG_WAY: nhãn line");
    check(st.kind(EventKind::kWrongWay).needs_snapshot,
          "WRONG_WAY: cần ảnh bằng chứng riêng");
    check(st.kind(EventKind::kWrongWay).first_hit_at_s == 5.0, "WRONG_WAY: mốc hit đầu");
    check(!st.allSettled(), "có vi phạm chưa bắn → allSettled() false");

    // Nhãn chỉ ghi lần đầu.
    st.addWrongWayObservation("LINE_KHAC", 6.0);
    checkEqInt(st.kind(EventKind::kWrongWay).hits, 2, "WRONG_WAY: hits tăng");
    checkEq(st.kindLabel(EventKind::kWrongWay), "REVERSE_DIRECTION",
            "WRONG_WAY: nhãn giữ lần đầu");
  }

  {
    // Cờ per-kind độc lập: posted kind này không ảnh hưởng kind kia.
    TrackPlateState st(21, 0.0);
    st.addWrongLaneObservation("CAR_TRUCK_LANE");
    st.addHelmetObservation(2);
    checkEqInt(st.kind(EventKind::kNoHelmet).detail, 2, "NO_HELMET: detail = số người");
    check(!st.kind(EventKind::kNoHelmet).needs_snapshot,
          "NO_HELMET dùng ảnh chung, không cần ảnh riêng");

    st.markPosted(EventKind::kWrongLane);
    check(st.kind(EventKind::kWrongLane).posted, "WRONG_LANE đã posted");
    check(!st.kind(EventKind::kNoHelmet).posted, "NO_HELMET chưa posted (độc lập)");
    check(!st.allSettled(), "còn NO_HELMET chưa bắn → chưa settled");
    st.markPosted(EventKind::kNoHelmet);
    check(st.allSettled(), "mọi kind active đã posted → allSettled()");
  }

  {
    // clearKind bỏ riêng 1 vế, các kind khác nguyên vẹn.
    TrackPlateState st(22, 0.0);
    st.addWrongWayObservation("REVERSE_DIRECTION", 1.0);
    st.addWrongLaneObservation("CAR_LANE");
    st.clearKind(EventKind::kWrongWay);
    checkEqInt(st.kind(EventKind::kWrongWay).hits, 0, "clearKind: WRONG_WAY về 0");
    checkEq(st.kindLabel(EventKind::kWrongWay), "", "clearKind: nhãn WRONG_WAY xoá");
    checkEqInt(st.kind(EventKind::kWrongLane).hits, 1, "clearKind: WRONG_LANE nguyên vẹn");
  }

  {
    // shouldForceDelete: track còn kind chưa bắn thì KHÔNG bị xoá sớm.
    TrackPlateState st(26, 0.0, 2);
    st.onEnterPolygon(0.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0, 0.9, 100.0, 40.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 2.0, 0.9, 100.0, 40.0);
    st.addWrongWayObservation("REVERSE_DIRECTION", 2.0);
    check(!st.shouldForceDelete(3.0), "còn WRONG_WAY chưa bắn → chưa xoá track");

    st.markPosted(EventKind::kPlate);
    check(!st.shouldForceDelete(3.0), "PLATE đã bắn nhưng WRONG_WAY chưa → vẫn giữ track");

    st.markPosted(EventKind::kWrongWay);
    check(st.shouldForceDelete(3.0), "mọi kind đã bắn → xoá track");
  }

  {
    // Trần tuyệt đối vẫn giữ: track kẹt vì kind không bao giờ posted vẫn bị xoá.
    TrackPlateState st(27, 0.0, 2);
    st.onEnterPolygon(0.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0, 0.9, 100.0, 40.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 2.0, 0.9, 100.0, 40.0);
    st.addWrongWayObservation("REVERSE_DIRECTION", 2.0);
    st.onLeavePolygon(3.0);
    check(st.shouldForceDelete(3.0 + p::kForceDeleteIdleS + 1.0),
          "kẹt ngoài zone quá kForceDeleteIdleS → vẫn xoá (không rò rỉ)");
    check(st.shouldForceDelete(p::kForceDeleteAgeS + 1.0),
          "quá kForceDeleteAgeS → vẫn xoá (không rò rỉ)");
  }

  {
    // emitPlate: normalize sẵn lúc chốt.
    TrackPlateState st(23, 0.0, 2);
    st.onEnterPolygon(0.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0, 0.9, 100.0, 40.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 2.0, 0.9, 100.0, 40.0);
    check(st.hasFinalPlate(), "emitPlate: đã chốt");
    checkEq(st.emitPlate(), p::normalizePlateForEmit(st.plate()),
            "emitPlate() khớp normalizePlateForEmit(plate())");
  }

  {
    // Rendezvous: vi phạm đến TRƯỚC biển.
    TrackPlateState st(24, 0.0, 2);
    st.addWrongWayObservation("REVERSE_DIRECTION", 1.0);
    check(st.wrongWayPairedAtS() == 0.0, "vi phạm trước: chưa paired (thiếu biển)");
    st.onEnterPolygon(2.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 2.0, 0.9, 100.0, 40.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 3.0, 0.9, 100.0, 40.0);
    check(st.hasFinalPlate(), "vi phạm trước: biển đã chốt");
    check(st.wrongWayPairedAtS() == 3.0, "vi phạm trước: chốt biển làm đủ cặp");
  }

  {
    // Rendezvous: biển đến TRƯỚC vi phạm.
    TrackPlateState st(25, 0.0, 2);
    st.onEnterPolygon(0.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 1.0, 0.9, 100.0, 40.0);
    st.addOcrReading(makeChars("14A12345"), "14A12345", 2.0, 0.9, 100.0, 40.0);
    check(st.hasFinalPlate(), "biển trước: đã chốt");
    st.addWrongWayObservation("REVERSE_DIRECTION", 7.0);
    check(st.wrongWayPairedAtS() == 7.0, "biển trước: vi phạm làm đủ cặp");
  }
}

// ---------------------------------------------------------------------------
// TrackPlateState — hình học WRONG_WAY
// ---------------------------------------------------------------------------
void testMotion() {
  std::printf("== motion / stationary ==\n");
  using vehicle::Point;

  {
    // Xe đỗ: anchor chỉ rung vài px quanh 1 tâm → phải coi là đứng yên.
    TrackPlateState st(7, 0.0);
    st.pushAnchorHistory(Point{100.0, 100.0});
    st.pushAnchorHistory(Point{101.0, 99.0});
    st.pushAnchorHistory(Point{99.0, 101.0});
    st.pushAnchorHistory(Point{100.0, 100.0});
    check(st.isStationary(), "xe đỗ rung bbox → isStationary() true");
  }

  {
    // Xe chạy thẳng: trôi đều theo một hướng.
    TrackPlateState st(8, 0.0);
    st.pushAnchorHistory(Point{100.0, 100.0});
    st.pushAnchorHistory(Point{110.0, 100.0});
    st.pushAnchorHistory(Point{120.0, 100.0});
    st.pushAnchorHistory(Point{130.0, 100.0});
    check(!st.isStationary(), "xe chạy → isStationary() false");
    const Point mv = st.motionVector();
    check(mv.x > 0.0, "motionVector().x > 0 khi xe chạy sang phải");
    check(std::fabs(mv.y) < 1e-6, "motionVector().y ~ 0 khi xe chạy ngang");
  }

  {
    // Chưa đủ history → chưa dám kết luận hướng.
    TrackPlateState st(9, 0.0);
    st.pushAnchorHistory(Point{0.0, 0.0});
    check(st.isStationary(), "1 anchor: coi như chưa đủ cơ sở → true");
    const Point mv = st.motionVector();
    check(mv.x == 0.0 && mv.y == 0.0, "1 anchor: motionVector() = {0,0}");
  }
}

// ---------------------------------------------------------------------------
// PlateRecognizer::collectReady
// ---------------------------------------------------------------------------
void testCollectReady() {
  std::printf("== PlateRecognizer ==\n");
  namespace p = vehicle::business::plate;

  vehicle::PlateConfig cfg;
  cfg.max_recognize_times = 3;
  cfg.send_mode = 2;  // bắn tất cả, kể cả UNKOWN
  p::PlateRecognizer rec(cfg);

  const uint64_t track = 100;
  rec.observeVehicle(track, 0, 0.9, /*in_zone=*/true, 1.0);
  for (int i = 0; i < 3; ++i) {
    rec.addOcrReading(track, makeChars("14A12345"), "14A12345", 1.0 + i, 0.9, 100.0, 40.0);
  }

  const std::vector<p::PendingEmit> ready = rec.collectReady(5.0);
  checkEqInt(static_cast<long long>(ready.size()), 1, "collectReady trả đúng 1 track");
  if (!ready.empty()) {
    checkEqInt(static_cast<long long>(ready[0].track_id), static_cast<long long>(track),
               "collectReady: track_id đúng");
    checkEq(ready[0].plate, "14A12345", "collectReady: biển đã normalize");
  }

  // Chưa commit thì lần gọi sau vẫn trả lại (retry throttle cho phép).
  rec.commitEmit(track, "14A12345", 5.0);
  const std::vector<p::PendingEmit> after = rec.collectReady(6.0);
  checkEqInt(static_cast<long long>(after.size()), 0,
             "sau commitEmit: track không còn trong collectReady");
}

// ---------------------------------------------------------------------------
// clearStaleWrongWay — quá hạn chỉ bỏ vế WRONG_WAY, track vẫn bắn được PLATE
// ---------------------------------------------------------------------------
void testClearStaleWrongWay() {
  std::printf("== clearStaleWrongWay ==\n");
  namespace p = vehicle::business::plate;

  vehicle::PlateConfig cfg;
  cfg.max_recognize_times = 2;
  cfg.send_mode = 2;
  p::PlateRecognizer rec(cfg);
  rec.setWrongWayTiming(/*settle_s=*/0.0, /*wait_pair_s=*/5.0);

  // Vạch cấm nằm ngang tại y=100, chiều CẤM là đi xuống (+y).
  std::vector<p::WrongWayLine> lines;
  {
    p::WrongWayLine ln;
    ln.a = vehicle::Point{0.0, 100.0};
    ln.b = vehicle::Point{200.0, 100.0};
    ln.direction = vehicle::Point{0.0, 1.0};
    ln.name = "REVERSE_DIRECTION";
    lines.push_back(ln);
  }

  const uint64_t track = 200;
  rec.observeVehicle(track, 0, 0.9, /*in_zone=*/true, 1.0);

  // Xe đi xuống, cắt vạch y=100 đúng chiều cấm. Cần đủ history để qua tầng lọc
  // isStationary + motionVector.
  rec.observeWrongWay(track, vehicle::Point{100.0, 80.0}, lines, 1.0);
  rec.observeWrongWay(track, vehicle::Point{100.0, 88.0}, lines, 1.0);
  rec.observeWrongWay(track, vehicle::Point{100.0, 96.0}, lines, 1.0);
  const bool hit = rec.observeWrongWay(track, vehicle::Point{100.0, 104.0}, lines, 1.0);
  check(hit, "xe cắt vạch đúng chiều cấm → ghi nhận WRONG_WAY");

  rec.addOcrReading(track, makeChars("14A12345"), "14A12345", 1.0, 0.9, 100.0, 40.0);
  rec.addOcrReading(track, makeChars("14A12345"), "14A12345", 2.0, 0.9, 100.0, 40.0);

  checkEqInt(static_cast<long long>(rec.clearStaleWrongWay(3.0)), 0,
             "chưa quá wait_pair_s: không bỏ vế nào");

  // Quá 5s kể từ lúc cắt vạch → bỏ RIÊNG vế WRONG_WAY.
  checkEqInt(static_cast<long long>(rec.clearStaleWrongWay(7.0)), 1,
             "quá wait_pair_s: bỏ đúng 1 vế WRONG_WAY");

  // Điểm mấu chốt của B1: track PHẢI còn sống và bắn được event PLATE.
  const std::vector<p::PendingEmit> ready = rec.collectReady(8.0);
  checkEqInt(static_cast<long long>(ready.size()), 1,
             "sau khi bỏ vế WRONG_WAY: track vẫn bắn được PLATE");
  if (!ready.empty()) {
    checkEqInt(static_cast<long long>(ready[0].wrong_way_hits), 0,
               "vế WRONG_WAY đã sạch, không bắn nhầm");
    checkEq(ready[0].plate, "14A12345", "biển vẫn nguyên vẹn");
  }
}

}  // namespace

int main() {
  std::printf("vehicle_business_tests\n\n");
  testRules();
  testDedupCache();
  testTrackPlateState();
  testKindLifecycle();
  testMotion();
  testCollectReady();
  testClearStaleWrongWay();

  std::printf("\n%d/%d case pass", g_total - g_failed, g_total);
  if (g_failed > 0) {
    std::printf(" — %d FAIL\n", g_failed);
    return 1;
  }
  std::printf(" — tất cả xanh\n");
  return 0;
}
