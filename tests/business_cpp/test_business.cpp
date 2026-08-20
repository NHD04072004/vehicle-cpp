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

  {
    DedupCache cache;
    check(cache.tryEmit(1, "14A12345"), "lần đầu (track 1) được bắn");
    check(!cache.tryEmit(1, "14A12345"), "cùng track + cùng biển bị chặn");
  }

  {
    // Hai TRACK KHÁC NHAU đọc ra cùng chuỗi biển — hai xe khác nhau, cả hai
    // đều phải được bắn. Rất phổ biến với 'UNKOWN' (biển không đọc được).
    DedupCache cache;
    check(cache.tryEmit(1, vehicle::business::plate::kUnknownPlate),
          "track 1 UNKOWN được bắn");
    check(cache.tryEmit(2, vehicle::business::plate::kUnknownPlate),
          "track 2 UNKOWN cũng phải được bắn (xe khác)");
    check(cache.tryEmit(3, "14A12345"), "track 3 biển khác được bắn");
    check(cache.tryEmit(4, "14A12345"), "track 4 cùng biển với track 3 vẫn phải bắn");
  }

  {
    // maxlen: entry cũ nhất bị đẩy ra, track cũ được bắn lại.
    DedupCache cache(2);
    check(cache.tryEmit(10, "A"), "cache(2): track 10");
    check(cache.tryEmit(11, "B"), "cache(2): track 11");
    check(cache.tryEmit(12, "C"), "cache(2): track 12 đẩy track 10 ra");
    check(cache.tryEmit(10, "A"), "cache(2): track 10 bắn lại được sau khi bị đẩy ra");
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
  rec.commitEmit(track, "14A12345");
  const std::vector<p::PendingEmit> after = rec.collectReady(6.0);
  checkEqInt(static_cast<long long>(after.size()), 0,
             "sau commitEmit: track không còn trong collectReady");
}

}  // namespace

int main() {
  std::printf("vehicle_business_tests\n\n");
  testRules();
  testDedupCache();
  testTrackPlateState();
  testMotion();
  testCollectReady();

  std::printf("\n%d/%d case pass", g_total - g_failed, g_total);
  if (g_failed > 0) {
    std::printf(" — %d FAIL\n", g_failed);
    return 1;
  }
  std::printf(" — tất cả xanh\n");
  return 0;
}
