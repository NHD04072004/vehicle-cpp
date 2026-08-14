#pragma once

#include <glib.h>
#include <gst/gst.h>

#include <map>
#include <set>
#include <string>
#include <vector>

#include "common/config.h"
#include "common/types.h"
#include "probes/plate_probe.h"

namespace vehicle {
namespace pipeline {

struct SourceSpec {
  std::string uri;
  std::string camera_code;
};

class Pipeline {
 public:
  Pipeline(const Config& config, probes::PlateProbe* probe);
  ~Pipeline();

  bool build(const std::vector<SourceSpec>& sources);
  bool start();
  void stop();
  bool pause();
  bool resume();

  // Runtime (gọi trên GLib main loop): add / remove / update theo camera_code.
  bool addSource(const Camera& camera);
  bool removeSource(const std::string& camera_code);
  bool updateSource(const Camera& camera);
  void applyCameraList(const std::vector<Camera>& cameras);

  unsigned int maxSources() const { return max_sources_; }

  void setMainLoop(GMainLoop* loop) { loop_ = loop; }

 private:
  struct SourceSlot {
    unsigned int source_id = 0;
    std::string camera_code;
    std::string uri;
    Camera camera;
    GstElement* element = nullptr;
    GstPad* mux_pad = nullptr;
  };

  GstElement* makeElement(const char* factory, const char* name);
  bool addSourceInternal(const Camera& camera, int preferred_id);
  bool removeSourceInternal(const std::string& camera_code, bool keep_slot_id);
  int allocateSourceId(int preferred_id);
  void configureRtsp(GstElement* source, const std::string& uri);
  bool buildSink(GstElement** first, GstElement** last);
  bool isSourceElement(GstObject* obj) const;
  static void onPadAdded(GstElement* src, GstPad* pad, gpointer data);
  static gboolean onBusMessage(GstBus* bus, GstMessage* msg, gpointer data);

  const Config& config_;
  probes::PlateProbe* probe_;
  GstElement* pipeline_ = nullptr;
  GstElement* muxer_ = nullptr;
  GMainLoop* loop_ = nullptr;
  guint bus_watch_id_ = 0;
  unsigned int max_sources_ = 0;
  bool playing_ = false;

  std::map<std::string, SourceSlot> slots_;  // camera_code → slot
  std::set<unsigned int> free_ids_;
};

}  // namespace pipeline
}  // namespace vehicle
