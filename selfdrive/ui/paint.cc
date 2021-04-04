#include "ui.hpp"

#include <assert.h>
#include <map>
#include <cmath>
#include <iostream>
#include "common/util.h"
#include "common/timing.h"
#include <algorithm>

#define NANOVG_GLES3_IMPLEMENTATION
#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"
#include "paint.hpp"
#include "sidebar.hpp"

// TODO: this is also hardcoded in common/transformations/camera.py
// TODO: choose based on frame input size
#ifdef QCOM2
const float y_offset = 150.0;
const float zoom = 1.1;
const mat3 intrinsic_matrix = (mat3){{
  2648.0, 0.0, 1928.0/2,
  0.0, 2648.0, 1208.0/2,
  0.0,   0.0,   1.0
}};
#else
const float y_offset = 0.0;
const float zoom = 2.35;
const mat3 intrinsic_matrix = (mat3){{
  910., 0., 1164.0/2,
  0., 910., 874.0/2,
  0.,   0.,   1.
}};
#endif

// Projects a point in car to space to the corresponding point in full frame
// image space.
bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, vertex_data *out) {
  const float margin = 500.0f;
  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(intrinsic_matrix, Ep);

  // Project.
  float x = KEp.v[0] / KEp.v[2];
  float y = KEp.v[1] / KEp.v[2];

  nvgTransformPoint(&out->x, &out->y, s->car_space_transform, x, y);
  return out->x >= -margin && out->x <= s->fb_w + margin && out->y >= -margin && out->y <= s->fb_h + margin;
}

static void ui_draw_text(const UIState *s, float x, float y, const char* string, float size, NVGcolor color, const char *font_name){
  nvgFontFace(s->vg, font_name);
  nvgFontSize(s->vg, size);
  nvgFillColor(s->vg, color);
  nvgText(s->vg, x, y, string, NULL);
}

static void draw_chevron(UIState *s, float x, float y, float sz, NVGcolor fillColor, NVGcolor glowColor) {
  // glow
  float g_xo = sz/5;
  float g_yo = sz/10;
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.35)+g_xo, y+sz+g_yo);
  nvgLineTo(s->vg, x, y-g_xo);
  nvgLineTo(s->vg, x-(sz*1.35)-g_xo, y+sz+g_yo);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, glowColor);
  nvgFill(s->vg);

  // chevron
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, x+(sz*1.25), y+sz);
  nvgLineTo(s->vg, x, y);
  nvgLineTo(s->vg, x-(sz*1.25), y+sz);
  nvgClosePath(s->vg);
  nvgFillColor(s->vg, fillColor);
  nvgFill(s->vg);
}

static void ui_draw_circle_image(const UIState *s, int x, int y, int size, const char *image, NVGcolor color, float img_alpha, int img_y = 0) {
  const int img_size = size * 1.5;
  nvgBeginPath(s->vg);
  nvgCircle(s->vg, x, y + (bdr_s * 4.5), size);
  nvgFillColor(s->vg, color);
  nvgFill(s->vg);
  ui_draw_image(s, {x - (img_size / 2), img_y ? img_y : y - (size / 4), img_size, img_size}, image, img_alpha);
}

static void ui_draw_circle_image(const UIState *s, int x, int y, int size, const char *image, bool active) {
  float bg_alpha = active ? 0.3f : 0.1f;
  float img_alpha = active ? 1.0f : 0.15f;
  ui_draw_circle_image(s, x, y, size, image, nvgRGBA(0, 0, 0, (255 * bg_alpha)), img_alpha);
}

static void draw_lead(UIState *s, int idx){
  // Draw lead car indicator
  const auto &lead = s->scene.lead_data[idx];
  auto [x, y] = s->scene.lead_vertices[idx];

  float fillAlpha = 0;
  float speedBuff = 10.;
  float leadBuff = 40.;
  float d_rel = lead.getDRel();
  float v_rel = lead.getVRel();

  if (d_rel < leadBuff) {
    fillAlpha = 255*(1.0-(d_rel/leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255*(-1*(v_rel/speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * zoom;
  x = std::clamp(x, 0.f, s->viz_rect.right() - sz / 2);
  y = std::fmin(s->viz_rect.bottom() - sz * .6,  y);
  draw_chevron(s, x, y, sz, COLOR_RED_ALPHA(fillAlpha), COLOR_YELLOW);
}

static void ui_draw_line(UIState *s, const line_vertices_data &vd, NVGcolor *color, NVGpaint *paint) {
  if (vd.cnt == 0) return;

  const vertex_data *v = &vd.v[0];
  nvgBeginPath(s->vg);
  nvgMoveTo(s->vg, v[0].x, v[0].y);
  for (int i = 1; i < vd.cnt; i++) {
    nvgLineTo(s->vg, v[i].x, v[i].y);
  }
  nvgClosePath(s->vg);
  if (color) {
    nvgFillColor(s->vg, *color);
  } else if (paint) {
    nvgFillPaint(s->vg, *paint);
  }
  nvgFill(s->vg);
}

static void draw_frame(UIState *s) {
  mat4 *out_mat;
  if (s->scene.frontview) {
    glBindVertexArray(s->frame_vao[1]);
    out_mat = &s->front_frame_mat;
  } else {
    glBindVertexArray(s->frame_vao[0]);
    out_mat = &s->rear_frame_mat;
  }
  glActiveTexture(GL_TEXTURE0);

  if (s->last_frame) {
    glBindTexture(GL_TEXTURE_2D, s->texture[s->last_frame->idx]->frame_tex);
#ifndef QCOM
    // this is handled in ion on QCOM
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, s->last_frame->width, s->last_frame->height,
                 0, GL_RGB, GL_UNSIGNED_BYTE, s->last_frame->addr);
#endif
  }

  glUseProgram(s->gl_shader->prog);
  glUniform1i(s->gl_shader->getUniformLocation("uTexture"), 0);
  glUniformMatrix4fv(s->gl_shader->getUniformLocation("uTransform"), 1, GL_TRUE, out_mat->v);

  assert(glGetError() == GL_NO_ERROR);
  glEnableVertexAttribArray(0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, (const void*)0);
  glDisableVertexAttribArray(0);
  glBindVertexArray(0);
}

static void ui_draw_vision_lane_lines(UIState *s) {
  const UIScene &scene = s->scene;
  // paint lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    NVGcolor color = nvgRGBAf(1.0, 1.0, 1.0, scene.lane_line_probs[i]);
    ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
  }

  // paint road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    NVGcolor color = nvgRGBAf(1.0, 0.0, 0.0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0));
    ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
  }

  // paint path
  int steerOverride = s->scene.car_state.getSteeringPressed();
  NVGpaint track_bg;
  if (s->scene.controls_state.getEnabled()) {
  // Draw colored track
    if (steerOverride) {
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
                                   COLOR_ENGAGEABLE, COLOR_ENGAGEABLE_ALPHA(120));
    } else {
      // color track with output scale
      int torque_scale = (int)fabs(510*(float)s->scene.output_scale);
      int red_lvl = fmin(255, torque_scale);
      int green_lvl = fmin(255, 510-torque_scale);
      track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
        nvgRGBA(          red_lvl,            green_lvl,  0, 255),
        nvgRGBA((int)(0.5*red_lvl), (int)(0.5*green_lvl), 0, 50));
    }
  } else {
    // Draw white vision track
    track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h*.4,
                                 COLOR_WHITE, COLOR_WHITE_ALPHA(120));
  }
  ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);
}

// Draw all world space objects.
static void ui_draw_world(UIState *s) {
  const UIScene *scene = &s->scene;
  // Don't draw on top of sidebar
  nvgScissor(s->vg, s->viz_rect.x, s->viz_rect.y, s->viz_rect.w, s->viz_rect.h);

  // Draw lane edges and vision/mpc tracks
  ui_draw_vision_lane_lines(s);

  // Draw lead indicators if openpilot is handling longitudinal
  //if (s->longitudinal_control) {
    if (scene->lead_data[0].getStatus()) {
      draw_lead(s, 0);
    }
    if (scene->lead_data[1].getStatus() && (std::abs(scene->lead_data[0].getDRel() - scene->lead_data[1].getDRel()) > 3.0)) {
      draw_lead(s, 1);
    }
  //}
  nvgResetScissor(s->vg);
}

static void ui_draw_vision_maxspeed(UIState *s) {
  const int SET_SPEED_NA = 255;
  float maxspeed = s->scene.controls_state.getVCruise();
  const bool is_cruise_set = maxspeed != 0 && maxspeed != SET_SPEED_NA;
  if (is_cruise_set && !s->is_metric) { maxspeed *= 0.6225; }

  const Rect rect = {s->viz_rect.x + (bdr_s * 2), int(s->viz_rect.y + (bdr_s * 1.5)), 184, 202};
  ui_fill_rect(s->vg, rect, COLOR_BLACK_ALPHA(100), 30.);
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(100), 10, 20.);

  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, rect.centerX(), 100, "SET", 45, COLOR_WHITE_ALPHA(is_cruise_set ? 200 : 100), "sans-regular");
  if (is_cruise_set) {
    const std::string maxspeed_str = std::to_string((int)std::nearbyint(maxspeed));
    ui_draw_text(s, rect.centerX(), 200, maxspeed_str.c_str(), 40 * 2.5, COLOR_WHITE, "sans-bold");
  } else {
    ui_draw_text(s, rect.centerX(), 200, "-", 40 * 2.5, COLOR_WHITE_ALPHA(100), "sans-semibold");
  }
}

static void ui_draw_vision_speed(UIState *s) {
  const float speed = std::max(0.0, s->scene.car_state.getVEgo() * (s->is_metric ? 3.6 : 2.2369363));
  const std::string speed_str = std::to_string((int)std::nearbyint(speed));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  ui_draw_text(s, s->viz_rect.centerX(), 240, speed_str.c_str(), 100 * 2.5, COLOR_WHITE, "sans-bold");
  ui_draw_text(s, s->viz_rect.centerX(), 320, s->is_metric ? "㎞/h" : "mph", 36 * 2.5, COLOR_YELLOW_ALPHA(200), "sans-regular");

  const int viz_blinker_w = 280;
  const int viz_blinker_x = s->viz_rect.centerX() - 140;
  const int viz_add = 50;

  // turning blinker sequential
  if(s->scene.leftBlinker) {
    nvgBeginPath(s->vg); // left1=50(viz_add) , y 105->100 , 210->200 , 315->300
    nvgMoveTo(s->vg, viz_blinker_x - viz_add                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - viz_add - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - viz_add                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 120 && s->scene.blinker_blinkingrate > 110)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left2=80
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*1.6)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*1.6) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*1.6)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 110 && s->scene.blinker_blinkingrate > 100)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left3=110
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*2.2)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*2.2) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*2.2)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 100 && s->scene.blinker_blinkingrate > 90)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left4=140
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*2.8)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*2.8) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*2.8)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 90 && s->scene.blinker_blinkingrate > 80)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left5=170
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*3.4)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*3.4) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*3.4)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 80 && s->scene.blinker_blinkingrate > 70)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left6=200
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*4)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*4) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*4)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 70 && s->scene.blinker_blinkingrate > 60)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left7=230
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*4.6)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*4.6) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*4.6)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 60 && s->scene.blinker_blinkingrate > 50)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left8=260
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*5.2)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*5.2) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*5.2)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 50 && s->scene.blinker_blinkingrate > 40)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left9=290
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*5.8)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*5.8) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*5.8)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 40 && s->scene.blinker_blinkingrate > 30)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // left10=320
    nvgMoveTo(s->vg, viz_blinker_x - (viz_add*6.4)                    , s->viz_rect.y + (header_h/4.2));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*6.4) - (viz_blinker_w/2), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x - (viz_add*6.4)                    , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 30 && s->scene.blinker_blinkingrate > 20)?180:0));
    nvgFill(s->vg);
  }
  if(s->scene.rightBlinker) {
    nvgBeginPath(s->vg); // right1=50
    nvgMoveTo(s->vg, viz_blinker_x + viz_add + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + viz_add + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + viz_add + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 120 && s->scene.blinker_blinkingrate > 110)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right2=80
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*1.6) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*1.6) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*1.6) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 110 && s->scene.blinker_blinkingrate > 100)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right3=110
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*2.2) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*2.2) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*2.2) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 100 && s->scene.blinker_blinkingrate > 90)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right4=140
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*2.8) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*2.8) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*2.8) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 90 && s->scene.blinker_blinkingrate > 80)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right5=170
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*3.4) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*3.4) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*3.4) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 80 && s->scene.blinker_blinkingrate > 70)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right6=200
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*4) + viz_blinker_w     , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*4) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*4) + viz_blinker_w       , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 70 && s->scene.blinker_blinkingrate > 60)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right7=230
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*4.6) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*4.6) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*4.6) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 60 && s->scene.blinker_blinkingrate > 50)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right8=260
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*5.2) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*5.2) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*5.2) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 50 && s->scene.blinker_blinkingrate > 40)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right9=290
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*5.8) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*5.8) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*5.8) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 40 && s->scene.blinker_blinkingrate > 30)?180:0));
    nvgFill(s->vg);

    nvgBeginPath(s->vg); // right10=320
    nvgMoveTo(s->vg, viz_blinker_x + (viz_add*6.4) + viz_blinker_w      , s->viz_rect.y + (header_h/4));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*6.4) + (viz_blinker_w*1.5), s->viz_rect.y + (header_h/2.1));
    nvgLineTo(s->vg, viz_blinker_x + (viz_add*6.4) + viz_blinker_w      , s->viz_rect.y + (header_h/1.4));
    nvgClosePath(s->vg);
    nvgFillColor(s->vg, COLOR_WARNING_ALPHA((s->scene.blinker_blinkingrate <= 30 && s->scene.blinker_blinkingrate > 20)?180:0));
    nvgFill(s->vg);
    }
  if(s->scene.leftBlinker || s->scene.rightBlinker) {
    s->scene.blinker_blinkingrate -= 5;
    if(s->scene.blinker_blinkingrate < 0) s->scene.blinker_blinkingrate = 120;
  }
}

static void ui_draw_vision_event(UIState *s) {
  const int viz_event_w = 220;
  const int viz_event_x = s->viz_rect.right() - (viz_event_w + bdr_s*3);
  const int viz_event_y = s->viz_rect.y + (bdr_s*4.5);

   // draw steering wheel
    float angleSteers = s->scene.car_state.getSteeringAngleDeg();
    int steerOverride = s->scene.car_state.getSteeringPressed();

    const int bg_wheel_size = 100;
    const int bg_wheel_x = viz_event_x + (viz_event_w - bg_wheel_size);
    const int bg_wheel_y = viz_event_y + (bg_wheel_size/2);
    const int img_wheel_size = bg_wheel_size*1.5;
    const int img_wheel_x = bg_wheel_x - (img_wheel_size/2);
    const int img_wheel_y = bg_wheel_y - (bdr_s*4.5);
    const float img_rotation = angleSteers/180*3.141592;
    float img_wheel_alpha = 0.1f;
    bool is_engaged = (s->status == STATUS_ENGAGED) && ! steerOverride;
    bool is_warning = (s->status == STATUS_WARNING);
    bool is_engageable = s->scene.controls_state.getEngageable();

    if (is_engaged || is_warning || is_engageable) {
      nvgBeginPath(s->vg);
      nvgCircle(s->vg, bg_wheel_x, (bg_wheel_y + (bdr_s*3)), bg_wheel_size);
      if (is_engaged) {
        nvgFillColor(s->vg, COLOR_ENGAGED_ALPHA(180));
      } else if (is_warning) {
        nvgFillColor(s->vg, COLOR_WARNING_ALPHA(180));
      } else if (is_engageable) {
        nvgFillColor(s->vg, COLOR_ENGAGEABLE_ALPHA(180));
      }
      nvgFill(s->vg);
      img_wheel_alpha = 1.0f;
    }
    nvgSave(s->vg);
    nvgTranslate(s->vg,bg_wheel_x, bg_wheel_y + (bdr_s*3));
    nvgRotate(s->vg,-img_rotation);
    nvgBeginPath(s->vg);
    NVGpaint imgPaint = nvgImagePattern(s->vg, img_wheel_x-bg_wheel_x, img_wheel_y-(bg_wheel_y + (bdr_s*3)), img_wheel_size, img_wheel_size, 0, s->images["wheel"], img_wheel_alpha);
    nvgRect(s->vg, img_wheel_x-bg_wheel_x, img_wheel_y-(bg_wheel_y + (bdr_s*3)), img_wheel_size, img_wheel_size);
    nvgFillPaint(s->vg, imgPaint);
    nvgFill(s->vg);
    nvgRestore(s->vg);
  }

static void ui_draw_vision_face(UIState *s) {
  const int face_size = 85;
  const int face_x = (s->viz_rect.x + face_size + (bdr_s*2));
  const int face_y = (s->viz_rect.bottom() - footer_h + ((footer_h - face_size) / 2));
  ui_draw_circle_image(s, face_x, face_y, face_size, "driver_face", s->scene.dmonitoring_state.getIsActiveMode());
}

static void ui_draw_vision_brake(UIState *s) {
  const int brake_size = 85;
  const int brake_x = (s->viz_rect.x + brake_size + (bdr_s*2));
  const int brake_y = (s->viz_rect.bottom() - footer_h + ((footer_h - brake_size) / 2));
  ui_draw_circle_image(s, brake_x + (brake_size*2), brake_y, brake_size, "brake_disc", s->scene.car_state.getBrakeLights());
}

static void ui_draw_vision_bsd_left(UIState *s) {
  const int bsd_size = 85;
  const int bsd_x = (s->viz_rect.x + bsd_size + (bdr_s*2));
  const int bsd_y = (s->viz_rect.bottom() - footer_h + ((footer_h - bsd_size) / 2));
  ui_draw_circle_image(s, bsd_x, bsd_y - (bsd_size*2), bsd_size, "bsd_l", s->scene.car_state.getLeftBlindspot());
}

static void ui_draw_vision_bsd_right(UIState *s) {
  const int bsd_size = 85;
  const int bsd_x = (s->viz_rect.x + bsd_size + (bdr_s*2));
  const int bsd_y = (s->viz_rect.bottom() - footer_h + ((footer_h - bsd_size) / 2));
  ui_draw_circle_image(s, bsd_x + (bsd_size*2), bsd_y - (bsd_size*2), bsd_size, "bsd_r", s->scene.car_state.getRightBlindspot());
}

static void ui_draw_driver_view(UIState *s) {
  s->sidebar_collapsed = true;
  const bool is_rhd = s->scene.is_rhd;
  const int width = 3 * s->viz_rect.w / 4;
  const Rect rect = {s->viz_rect.centerX() - width / 2, s->viz_rect.y, width, s->viz_rect.h};  // x, y, w, h
  const Rect valid_rect = {is_rhd ? rect.right() - rect.h / 2 : rect.x, rect.y, rect.h / 2, rect.h};

  // blackout
  const int blackout_x = is_rhd ? rect.x : valid_rect.right();
  const int blackout_w = rect.w - valid_rect.w;
  NVGpaint gradient = nvgLinearGradient(s->vg, blackout_x, rect.y, blackout_x + blackout_w, rect.y,
                                        COLOR_BLACK_ALPHA(is_rhd ? 255 : 0), COLOR_BLACK_ALPHA(is_rhd ? 0 : 255));
  ui_fill_rect(s->vg, {blackout_x, rect.y, blackout_w, rect.h}, gradient);
  ui_fill_rect(s->vg, {blackout_x, rect.y, blackout_w, rect.h}, COLOR_BLACK_ALPHA(144));
  // border
  ui_draw_rect(s->vg, rect, bg_colors[STATUS_OFFROAD], 1);

  const bool face_detected = s->scene.driver_state.getFaceProb() > 0.4;
  if (face_detected) {
    auto fxy_list = s->scene.driver_state.getFacePosition();
    float face_x = fxy_list[0];
    float face_y = fxy_list[1];
    int fbox_x = valid_rect.centerX() + (is_rhd ? face_x : -face_x) * valid_rect.w;
    int fbox_y = valid_rect.centerY() + face_y * valid_rect.h;

    float alpha = 0.2;
    if (face_x = std::abs(face_x), face_y = std::abs(face_y); face_x <= 0.35 && face_y <= 0.4)
      alpha = 0.8 - (face_x > face_y ? face_x : face_y) * 0.6 / 0.375;

    const int box_size = 0.6 * rect.h / 2;
    ui_draw_rect(s->vg, {fbox_x - box_size / 2, fbox_y - box_size / 2, box_size, box_size}, nvgRGBAf(1.0, 1.0, 1.0, alpha), 10, 35.);
  }

  // draw face icon / brake / bsd_left / bsd_right
  const int face_size = 85;
  const int icon_x = is_rhd ? rect.right() - face_size - (bdr_s*2) : rect.x + face_size + (bdr_s*2);
  const int icon_y = rect.bottom() - face_size;
  ui_draw_circle_image(s, icon_x,                 icon_y,                 face_size, "driver_face", face_detected);
  ui_draw_circle_image(s, icon_x + (face_size*2), icon_y,                 face_size, "brake_disc", s->scene.car_state.getBrakeLights());
  ui_draw_circle_image(s, icon_x,                 icon_y - (face_size*2), face_size, "bsd_l", s->scene.car_state.getLeftBlindspot());
  ui_draw_circle_image(s, icon_x + (face_size*2), icon_y - (face_size*2), face_size, "bsd_r", s->scene.car_state.getRightBlindspot());
}

static void ui_draw_vision_header(UIState *s) {
  NVGpaint gradient = nvgLinearGradient(s->vg, s->viz_rect.x,
                        s->viz_rect.y+(header_h-(header_h/2.5)),
                        s->viz_rect.x, s->viz_rect.y+header_h,
                        nvgRGBAf(0,0,0,0.45), nvgRGBAf(0,0,0,0));

  ui_fill_rect(s->vg, {s->viz_rect.x, s->viz_rect.y, s->viz_rect.w, header_h}, gradient);

  ui_draw_vision_maxspeed(s);
  ui_draw_vision_speed(s);
  ui_draw_vision_event(s);
}

//BB START: functions added for the display of various items

static int bb_ui_draw_measure(UIState *s, const char* bb_value, const char* bb_label, int bb_x, int bb_y,
                              NVGcolor bb_valueColor, NVGcolor bb_labelColor, int bb_valueFontSize, int bb_labelFontSize) {
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  //print value
  nvgFontFace(s->vg, "sans-semibold");
  nvgFontSize(s->vg, bb_valueFontSize);
  nvgFillColor(s->vg, bb_valueColor);
  nvgText(s->vg, bb_x, bb_y+ (int)(bb_valueFontSize), bb_value, NULL);

  //print label
  nvgFontFace(s->vg, "sans-regular");
  nvgFontSize(s->vg, bb_labelFontSize);
  nvgFillColor(s->vg, bb_labelColor);
  nvgText(s->vg, bb_x, bb_y + (int)(bb_valueFontSize) + (int)(bb_labelFontSize), bb_label, NULL);

  return (int)((bb_valueFontSize + bb_labelFontSize));
}

static void bb_ui_draw_measures_right(UIState *s, int bb_x, int bb_y, int bb_w) {
  const UIScene *scene = &s->scene;
  int bb_rx = bb_x + (int)(bb_w/2);
  int bb_ry = bb_y;
  int bb_h = 5;
  NVGcolor lab_color = COLOR_WHITE_ALPHA(200);
  int value_fontSize=65;
  int label_fontSize=35;

  //add CPU temperature average
  if (true) {
    char val_str[16];
    NVGcolor val_color = COLOR_ENGAGED;
      //show Orange if more than 70℃
      if((int)((scene->cpuTempAvg)) >= 70) {
        val_color = COLOR_WARNING;
      }
      //show Red if more than 80℃
      if((int)((scene->cpuTempAvg)) >= 80) {
        val_color = COLOR_RED_ALPHA(200);
      }
    snprintf(val_str, sizeof(val_str), "%.0f℃", (round((scene->cpuTempAvg))));
    bb_h += bb_ui_draw_measure(s, val_str, "CPU 온도", bb_rx, bb_ry, val_color, lab_color, value_fontSize, label_fontSize);
    bb_ry = bb_y + bb_h;
  }

  //add visual radar relative distance
  if (true) {
    char val_str[16];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (scene->lead_data[0].getStatus()) {
      //show Orange if less than 15ｍ
      if((int)(scene->lead_data[0].getDRel()) < 15) {
        val_color = COLOR_WARNING;
      }
      //show Red if less than 5ｍ
      if((int)(scene->lead_data[0].getDRel()) < 5) {
        val_color = COLOR_RED_ALPHA(200);
      }
      snprintf(val_str, sizeof(val_str), "%dｍ", (int)scene->lead_data[0].getDRel());
    } else {
      snprintf(val_str, sizeof(val_str), "-");
    }
    bb_h += bb_ui_draw_measure(s, val_str, "앞차 거리차", bb_rx, bb_ry, val_color, lab_color, value_fontSize, label_fontSize);
    bb_ry = bb_y + bb_h;
  }

  //add visual radar relative speed
  if (true) {
    char val_str[16];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (scene->lead_data[0].getStatus()) {
      //show Orange if negative speed
      if((int)(scene->lead_data[0].getVRel()) < 0) {
        val_color = COLOR_WARNING;
      }
      //show Red if positive speed
      if((int)(scene->lead_data[0].getVRel()) < -5) {
        val_color = COLOR_RED_ALPHA(200);
      }
      snprintf(val_str, sizeof(val_str), "%d㎞", (int)(scene->lead_data[0].getVRel() * 3.6 + 0.5));
    } else {
      snprintf(val_str, sizeof(val_str), "-");
    }
    bb_h +=bb_ui_draw_measure(s, val_str, "앞차 속도차", bb_rx, bb_ry, val_color, lab_color, value_fontSize, label_fontSize);
    bb_ry = bb_y + bb_h;
  }

  //add steering angle degree
  if (true) {
    char val_str[16];
    NVGcolor val_color = COLOR_ENGAGED;
    float angleSteers = s->scene.car_state.getSteeringAngleDeg();
      //show Orange if more than 30 degree
      if(((int)(angleSteers) < -30) || ((int)(angleSteers) > 30)) {
        val_color = COLOR_WARNING;
      }
      //show Red if more than 90 degree
      if(((int)(angleSteers) < -90) || ((int)(angleSteers) > 90)) {
        val_color = COLOR_RED_ALPHA(200);
      }
      // steering is in degree
      snprintf(val_str, sizeof(val_str), "%.1f˚",(angleSteers));
    bb_h += bb_ui_draw_measure(s, val_str, "핸들 조향각", bb_rx, bb_ry, val_color, lab_color, value_fontSize, label_fontSize);
    bb_ry = bb_y + bb_h;
  }

  //add desired steering angle degree
  if (true) {
    char val_str[16];
    NVGcolor val_color = COLOR_ENGAGED;
    float angleSteersDes  = s->scene.controls_state.getSteeringAngleDesiredDeg();
    if (scene->controls_state.getEnabled()) {
      //show Orange if more than 30 degree
      if(((int)(angleSteersDes) < -30) || ((int)(angleSteersDes) > 30)) {
        val_color = COLOR_WARNING;
      }
      //show Red if more than 90 degree
      if(((int)(angleSteersDes) < -90) || ((int)(angleSteersDes) > 90)) {
        val_color = COLOR_RED_ALPHA(200);
      }
      // steering is in degree
      snprintf(val_str, sizeof(val_str), "%.1f˚",(angleSteersDes));
    } else {
      snprintf(val_str, sizeof(val_str), "-");
    }
    bb_h += bb_ui_draw_measure(s, val_str, "OP 조향각", bb_rx, bb_ry, val_color, lab_color, value_fontSize, label_fontSize);
    bb_ry = bb_y + bb_h;
  }

  //add LateralControl
  if (true) {
    char val_str[16];
    NVGcolor val_color = COLOR_WHITE_ALPHA(200);
    if (s->lat_control_pid == 1) {
      snprintf(val_str, sizeof(val_str), "PID");
    } else if (s->lat_control_indi == 1) {
      snprintf(val_str, sizeof(val_str), "INDI");
    } else if (s->lat_control_lqr == 1) {
      snprintf(val_str, sizeof(val_str), "LQR");
    }
    bb_h += bb_ui_draw_measure(s, val_str, "조향로직", bb_rx, bb_ry, val_color, lab_color, value_fontSize, label_fontSize);
    bb_ry = bb_y + bb_h;
  }

  //finally draw the frame
  bb_h += 20;
    nvgBeginPath(s->vg);
    nvgRoundedRect(s->vg, bb_x, bb_y, bb_w, bb_h, 20);
    nvgStrokeColor(s->vg, COLOR_WHITE_ALPHA(80));
    nvgStrokeWidth(s->vg, 6);
    nvgStroke(s->vg);
}

static void bb_ui_draw_UI(UIState *s) {
  const int bb_dmr_w = 180;
  const int bb_dmr_x = 1920 - bb_dmr_w - (bdr_s*4.5);
  const int bb_dmr_y = (s->viz_rect.y + (bdr_s*4.5)) + 190;

  bb_ui_draw_measures_right(s, bb_dmr_x, bb_dmr_y, bb_dmr_w);
}

static void bb_ui_draw_tpms(UIState *s) {
  int viz_tpms_w = 240;
  int viz_tpms_h = 160;
  int viz_tpms_x = s->viz_rect.x + s->viz_rect.w - 270;
  int viz_tpms_y = s->viz_rect.x + 875;
  char tpmsFl[32];
  char tpmsFr[32];
  char tpmsRl[32];
  char tpmsRr[32];

  // Draw Border
  const Rect rect = {viz_tpms_x, viz_tpms_y, viz_tpms_w, viz_tpms_h};
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(80), 5, 20);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);
  const int pos_x = viz_tpms_x + (viz_tpms_w / 2);
  const int pos_y = 985;
  const int pos_add = 50;
  const int fontsize = 60;

  ui_draw_text(s, pos_x, pos_y+pos_add, "TPMS (psi)", fontsize-20, COLOR_WHITE_ALPHA(200), "sans-regular");
  snprintf(tpmsFl, sizeof(tpmsFl), "%.0f", s->scene.tpmsFl);
  snprintf(tpmsFr, sizeof(tpmsFr), "%.0f", s->scene.tpmsFr);
  snprintf(tpmsRl, sizeof(tpmsRl), "%.0f", s->scene.tpmsRl);
  snprintf(tpmsRr, sizeof(tpmsRr), "%.0f", s->scene.tpmsRr);

  if (s->scene.tpmsFl < 34) {
    ui_draw_text(s, pos_x - pos_add, pos_y-pos_add, tpmsFl, fontsize, COLOR_RED_ALPHA(200), "sans-bold");
  } else if (s->scene.tpmsFl > 50) {
    ui_draw_text(s, pos_x - pos_add, pos_y-pos_add, "-", fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else {
    ui_draw_text(s, pos_x - pos_add, pos_y-pos_add, tpmsFl, fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  }
  if (s->scene.tpmsFr < 34) {
    ui_draw_text(s, pos_x + pos_add, pos_y-pos_add, tpmsFr, fontsize, COLOR_RED_ALPHA(200), "sans-bold");
  } else if (s->scene.tpmsFr > 50) {
    ui_draw_text(s, pos_x + pos_add, pos_y-pos_add, "-", fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else {
    ui_draw_text(s, pos_x + pos_add, pos_y-pos_add, tpmsFr, fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  }
  if (s->scene.tpmsRl < 34) {
    ui_draw_text(s, pos_x - pos_add, pos_y, tpmsRl, fontsize, COLOR_RED_ALPHA(200), "sans-bold");
  } else if (s->scene.tpmsRl > 50) {
    ui_draw_text(s, pos_x - pos_add, pos_y, "-", fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else {
    ui_draw_text(s, pos_x - pos_add, pos_y, tpmsRl, fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  }
  if (s->scene.tpmsRr < 34) {
    ui_draw_text(s, pos_x + pos_add, pos_y, tpmsRr, fontsize, COLOR_RED_ALPHA(200), "sans-bold");
  } else if (s->scene.tpmsRr > 50) {
    ui_draw_text(s, pos_x + pos_add, pos_y, "-", fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  } else {
    ui_draw_text(s, pos_x + pos_add, pos_y, tpmsRr, fontsize, COLOR_WHITE_ALPHA(200), "sans-semibold");
  }
}

static void bb_ui_draw_gear(UIState *s) {
  UIScene &scene = s->scene;
  int viz_gear_w = 120;
  int viz_gear_h = 120;
  int viz_gear_x = s->viz_rect.x + s->viz_rect.w - 400;
  int viz_gear_y = s->viz_rect.y + (bdr_s*4.5) + 870;
  int getGear = int(scene.getGearShifter);
  char gear_msg[32];

  // Draw Border
  const Rect rect = {viz_gear_x, viz_gear_y, viz_gear_w, viz_gear_h};
  ui_draw_rect(s->vg, rect, COLOR_WHITE_ALPHA(80), 5, 20);
  NVGcolor Color = COLOR_WHITE_ALPHA(200);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  const int pos_x = viz_gear_x + (viz_gear_w / 2);
  const int pos_y = 985;
  const int pos_add = 50;
  const int fontsize = 60;
  ui_draw_text(s, pos_x, pos_y + pos_add, "Gear", fontsize - 20, Color, "sans-regular");

  switch( getGear ) {
    case 1 : strcpy( gear_msg, "P" ); break;
    case 2 : strcpy( gear_msg, "D" ); Color = COLOR_ENGAGED; break;
    case 3 : strcpy( gear_msg, "N" ); break;
    case 4 : strcpy( gear_msg, "R" ); Color = COLOR_RED_ALPHA(200); break;
    default: sprintf( gear_msg, "%d", getGear ); break;
  }
    ui_draw_text(s, pos_x, pos_y, gear_msg, fontsize + 20, Color, "sans-semibold");
}

//BB END: functions added for the display of various items

static void ui_draw_vision_footer(UIState *s) {
  ui_draw_vision_face(s);
  ui_draw_vision_brake(s);
  ui_draw_vision_bsd_left(s);
  ui_draw_vision_bsd_right(s);
  bb_ui_draw_UI(s);
  bb_ui_draw_tpms(s);
  bb_ui_draw_gear(s);
}

static float get_alert_alpha(float blink_rate) {
  return 0.375 * cos((millis_since_boot() / 1000) * 2 * M_PI * blink_rate) + 0.625;
}

static void ui_draw_vision_alert(UIState *s) {
  static std::map<cereal::ControlsState::AlertSize, const int> alert_size_map = {
      {cereal::ControlsState::AlertSize::SMALL, 241},
      {cereal::ControlsState::AlertSize::MID, 390},
      {cereal::ControlsState::AlertSize::FULL, s->fb_h}};
  const UIScene *scene = &s->scene;
  bool longAlert1 = scene->alert_text1.length() > 15;

  NVGcolor color = bg_colors[s->status];
  color.a *= get_alert_alpha(scene->alert_blinking_rate);
  const int alr_h = alert_size_map[scene->alert_size] + bdr_s;
  const Rect rect = {.x = s->viz_rect.x - bdr_s,
                  .y = s->fb_h - alr_h,
                  .w = s->viz_rect.w + (bdr_s * 2),
                  .h = alr_h};

  ui_fill_rect(s->vg, rect, color);
  ui_fill_rect(s->vg, rect, nvgLinearGradient(s->vg, rect.x, rect.y, rect.x, rect.bottom(),
                                            nvgRGBAf(0.0, 0.0, 0.0, 0.05), nvgRGBAf(0.0, 0.0, 0.0, 0.35)));

  nvgFillColor(s->vg, COLOR_WHITE);
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BASELINE);

  if (scene->alert_size == cereal::ControlsState::AlertSize::SMALL) {
    ui_draw_text(s, rect.centerX(), rect.centerY() + 15, scene->alert_text1.c_str(), 40*2, COLOR_WHITE, "sans-semibold");
  } else if (scene->alert_size == cereal::ControlsState::AlertSize::MID) {
    ui_draw_text(s, rect.centerX(), rect.centerY() - 45, scene->alert_text1.c_str(), 48*2, COLOR_WHITE, "sans-bold");
    ui_draw_text(s, rect.centerX(), rect.centerY() + 75, scene->alert_text2.c_str(), 36*2, COLOR_WHITE, "sans-regular");
  } else if (scene->alert_size == cereal::ControlsState::AlertSize::FULL) {
    nvgFontSize(s->vg, (longAlert1?72:96)*2.5);
    nvgFontFace(s->vg, "sans-bold");
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
    nvgTextBox(s->vg, rect.x, rect.y+(longAlert1?360:420), rect.w-60, scene->alert_text1.c_str(), NULL);
    nvgFontSize(s->vg, 48*2.5);
    nvgFontFace(s->vg,  "sans-regular");
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);
    nvgTextBox(s->vg, rect.x, rect.h-(longAlert1?300:360), rect.w-60, scene->alert_text2.c_str(), NULL);
  }
}

static void ui_draw_vision_frame(UIState *s) {
  // Draw video frames
  glEnable(GL_SCISSOR_TEST);
  glViewport(s->video_rect.x, s->video_rect.y, s->video_rect.w, s->video_rect.h);
  glScissor(s->viz_rect.x, s->viz_rect.y, s->viz_rect.w, s->viz_rect.h);
  draw_frame(s);
  glDisable(GL_SCISSOR_TEST);

  glViewport(0, 0, s->fb_w, s->fb_h);
}

static void ui_draw_vision(UIState *s) {
  const UIScene *scene = &s->scene;
  if (!scene->frontview) {
    // Draw augmented elements
    if (scene->world_objects_visible) {
      ui_draw_world(s);
    }
    // Set Speed, Current Speed, Status/Events
    ui_draw_vision_header(s);
    if (scene->alert_size == cereal::ControlsState::AlertSize::NONE) {
      ui_draw_vision_footer(s);
    }
  } else {
    ui_draw_driver_view(s);
  }
}

static void ui_draw_background(UIState *s) {
  const NVGcolor color = bg_colors[s->status];
  glClearColor(color.r, color.g, color.b, 1.0);
  glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
}

void ui_draw(UIState *s) {
  s->viz_rect = Rect{bdr_s, bdr_s, s->fb_w - 2 * bdr_s, s->fb_h - 2 * bdr_s};
  if (!s->sidebar_collapsed) {
    s->viz_rect.x += sbr_w;
    s->viz_rect.w -= sbr_w;
  }

  const bool draw_alerts = s->started && s->active_app == cereal::UiLayoutState::App::NONE;
  const bool draw_vision = draw_alerts && s->vipc_client->connected;

  // GL drawing functions
  ui_draw_background(s);
  if (draw_vision) {
    ui_draw_vision_frame(s);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glViewport(0, 0, s->fb_w, s->fb_h);

  // NVG drawing functions - should be no GL inside NVG frame
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  ui_draw_sidebar(s);
  if (draw_vision) {
    ui_draw_vision(s);
  }

  if (draw_alerts && s->scene.alert_size != cereal::ControlsState::AlertSize::NONE) {
    ui_draw_vision_alert(s);
  }
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha){
  nvgBeginPath(s->vg);
  NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
  nvgRect(s->vg, r.x, r.y, r.w, r.h);
  nvgFillPaint(s->vg, imgPaint);
  nvgFill(s->vg);
}

void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius) {
  nvgBeginPath(vg);
  radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  nvgStrokeColor(vg, color);
  nvgStrokeWidth(vg, width);
  nvgStroke(vg);
}

static inline void fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor *color, const NVGpaint *paint, float radius) {
  nvgBeginPath(vg);
  radius > 0? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
  if (color) nvgFillColor(vg, *color);
  if (paint) nvgFillPaint(vg, *paint);
  nvgFill(vg);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius) {
  fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius){
  fill_rect(vg, r, nullptr, &paint, radius);
}

static const char frame_vertex_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "in vec4 aPosition;\n"
  "in vec4 aTexCoord;\n"
  "uniform mat4 uTransform;\n"
  "out vec4 vTexCoord;\n"
  "void main() {\n"
  "  gl_Position = uTransform * aPosition;\n"
  "  vTexCoord = aTexCoord;\n"
  "}\n";

static const char frame_fragment_shader[] =
#ifdef NANOVG_GL3_IMPLEMENTATION
  "#version 150 core\n"
#else
  "#version 300 es\n"
#endif
  "precision mediump float;\n"
  "uniform sampler2D uTexture;\n"
  "in vec4 vTexCoord;\n"
  "out vec4 colorOut;\n"
  "void main() {\n"
  "  colorOut = texture(uTexture, vTexCoord.xy);\n"
  "}\n";

static const mat4 device_transform = {{
  1.0,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

// frame from 4/3 to 16/9 display
static const mat4 full_to_wide_frame_transform = {{
  .75,  0.0, 0.0, 0.0,
  0.0,  1.0, 0.0, 0.0,
  0.0,  0.0, 1.0, 0.0,
  0.0,  0.0, 0.0, 1.0,
}};

void ui_nvg_init(UIState *s) {
  // init drawing
#ifdef QCOM
  // on QCOM, we enable MSAA
  s->vg = nvgCreate(0);
#else
  s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
#endif
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/NanumGothic.ttf"},
      {"sans-semibold", "../assets/fonts/NanumGothicBold.ttf"},
      {"sans-bold", "../assets/fonts/NanumGothicExtraBold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }

  // init images
  std::vector<std::pair<const char *, const char *>> images = {
      {"wheel", "../assets/img_chffr_wheel.png"},
      {"trafficSign_turn", "../assets/img_trafficSign_turn.png"},
      {"driver_face", "../assets/img_driver_face.png"},
      {"brake_disc", "../assets/img_brake_disc.png"},
      {"bsd_l", "../assets/img_bsd_l.png"},
      {"bsd_r", "../assets/img_bsd_r.png"},
      {"button_settings", "../assets/images/button_settings.png"},
      {"button_home", "../assets/images/button_home.png"},
      {"battery", "../assets/images/battery.png"},
      {"battery_charging", "../assets/images/battery_charging.png"},
      {"network_0", "../assets/images/network_0.png"},
      {"network_1", "../assets/images/network_1.png"},
      {"network_2", "../assets/images/network_2.png"},
      {"network_3", "../assets/images/network_3.png"},
      {"network_4", "../assets/images/network_4.png"},
      {"network_5", "../assets/images/network_5.png"},
  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }

  // init gl
  s->gl_shader = std::make_unique<GLShader>(frame_vertex_shader, frame_fragment_shader);
  GLint frame_pos_loc = glGetAttribLocation(s->gl_shader->prog, "aPosition");
  GLint frame_texcoord_loc = glGetAttribLocation(s->gl_shader->prog, "aTexCoord");

  glViewport(0, 0, s->fb_w, s->fb_h);

  glDisable(GL_DEPTH_TEST);

  assert(glGetError() == GL_NO_ERROR);

  for(int i = 0; i < 2; i++) {
    float x1, x2, y1, y2;
    if (i == 1) {
      // flip horizontally so it looks like a mirror
      x1 = 0.0;
      x2 = 1.0;
      y1 = 1.0;
      y2 = 0.0;
    } else {
      x1 = 1.0;
      x2 = 0.0;
      y1 = 1.0;
      y2 = 0.0;
    }
    const uint8_t frame_indicies[] = {0, 1, 2, 0, 2, 3};
    const float frame_coords[4][4] = {
      {-1.0, -1.0, x2, y1}, //bl
      {-1.0,  1.0, x2, y2}, //tl
      { 1.0,  1.0, x1, y2}, //tr
      { 1.0, -1.0, x1, y1}, //br
    };

    glGenVertexArrays(1, &s->frame_vao[i]);
    glBindVertexArray(s->frame_vao[i]);
    glGenBuffers(1, &s->frame_vbo[i]);
    glBindBuffer(GL_ARRAY_BUFFER, s->frame_vbo[i]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(frame_coords), frame_coords, GL_STATIC_DRAW);
    glEnableVertexAttribArray(frame_pos_loc);
    glVertexAttribPointer(frame_pos_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(frame_coords[0]), (const void *)0);
    glEnableVertexAttribArray(frame_texcoord_loc);
    glVertexAttribPointer(frame_texcoord_loc, 2, GL_FLOAT, GL_FALSE,
                          sizeof(frame_coords[0]), (const void *)(sizeof(float) * 2));
    glGenBuffers(1, &s->frame_ibo[i]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s->frame_ibo[i]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(frame_indicies), frame_indicies, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER,0);
    glBindVertexArray(0);
  }

  s->video_rect = Rect{bdr_s, bdr_s, s->fb_w - 2 * bdr_s, s->fb_h - 2 * bdr_s};
  float zx = zoom * 2 * intrinsic_matrix.v[2] / s->video_rect.w;
  float zy = zoom * 2 * intrinsic_matrix.v[5] / s->video_rect.h;

  const mat4 frame_transform = {{
    zx, 0.0, 0.0, 0.0,
    0.0, zy, 0.0, -y_offset / s->video_rect.h * 2,
    0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0,
  }};

  s->front_frame_mat = matmul(device_transform, full_to_wide_frame_transform);
  s->rear_frame_mat = matmul(device_transform, frame_transform);

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  nvgTranslate(s->vg, s->video_rect.x + s->video_rect.w / 2, s->video_rect.y + s->video_rect.h / 2 + y_offset);

  // 2) Apply same scaling as video
  nvgScale(s->vg, zoom, zoom);

  // 3) Put (0, 0) in top left corner of video
  nvgTranslate(s->vg, -intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);

  nvgCurrentTransform(s->vg, s->car_space_transform);
  nvgResetTransform(s->vg);
}
