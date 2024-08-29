#pragma once
#include <rack.hpp>


using namespace rack;

// Declare the Plugin, defined in plugin.cpp
extern Plugin* pluginInstance;

// Declare each Model, defined in each module source file
extern Model* modelCosmos;
extern Model* modelGomaII;
extern Model* modelSlewLFO;

/** When triggered, holds a high value for a specified time before going low again */
struct PulseGenerator_4 {
	simd::float_4 remaining = 0.f;

	/** Immediately disables the pulse */
	void reset() {
		remaining = 0.f;
	}

	/** Advances the state by `deltaTime`. Returns whether the pulse is in the HIGH state. */
	simd::float_4 process(float deltaTime) {

		simd::float_4 mask = (remaining > 0.f);

		remaining -= ifelse(mask, deltaTime, 0.f);
		return ifelse(mask, simd::float_4::mask(), 0.f);
	}

	/** Begins a trigger with the given `duration`. */
	void trigger(simd::float_4 mask, float duration = 1e-3f) {
		// Keep the previous pulse if the existing pulse will be held longer than the currently requested one.
		remaining = ifelse(mask & (duration > remaining), duration, remaining);
	}
};

struct BooleanTrigger_4 {
	simd::float_4 previousMask = 0.f;

	simd::float_4 process(simd::float_4 mask) {
		simd::float_4 result = simd::andnot(previousMask, mask);
		previousMask = mask;
		return result;
	}
};

struct GoldPort : app::SvgPort {
	GoldPort() {
		setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/GoldPort.svg")));
	}
};

struct CKSSNarrow : app::SvgSwitch {
	CKSSNarrow() {
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/SwitchNarrow_0.svg")));
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/SwitchNarrow_2.svg")));
	}
};

struct HexnutKnobBlack : app::SvgKnob {
	widget::SvgWidget* bg;

	HexnutKnobBlack() {
		minAngle = -0.8 * M_PI;
		maxAngle = 0.8 * M_PI;

		bg = new widget::SvgWidget;
		fb->addChildBelow(bg, tw);

		setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/HexTinyKnobBlack.svg")));
		bg->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/HexTinyKnobBlack_bg.svg")));
	}
};

struct DoepferSwitch : app::SvgSwitch {
	DoepferSwitch() {
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/switch_0.svg")));
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/switch_1.svg")));
	}
};

struct BlackNoiseLed : TSvgLight<RedGreenBlueLight> {
	static constexpr float backgroundGrey = 77.f / 255.f;
	BlackNoiseLed() {

	}

	void draw(const DrawArgs& args) override {}
	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer == 1) {

			if (!sw->svg)
				return;

			if (module && !module->isBypassed()) {

				// when LED is off, draw the background (grey value #4d4d4d), but if on then progressively blend away to zero
				float backgroundFactor = std::max(0.f, 1.f - color.a) * backgroundGrey;
				if (backgroundFactor > 0.f) {
					NVGcolor c = nvgRGBf(backgroundFactor, backgroundFactor, backgroundFactor);

					for (auto s = sw->svg->handle->shapes; s; s = s->next) {
						s->fill.color = ((int)(c.a * 255) << 24) + (((int)(c.b * 255)) << 16) + (((int)(c.g * 255)) << 8) + (int)(c.r * 255);
						s->fill.type = NSVG_PAINT_COLOR;
					}

					nvgGlobalCompositeBlendFunc(args.vg, NVG_ONE_MINUS_DST_COLOR, NVG_ONE);
					svgDraw(args.vg, sw->svg->handle);
				}

				// main RGB color
				const int fillColor = ((int)(color.a * 255) << 24) + (((int)(color.b * 255)) << 16) + (((int)(color.g * 255)) << 8) + (int)(color.r * 255);
				for (auto s = sw->svg->handle->shapes; s; s = s->next) {
					s->fill.color = fillColor;
					s->fill.type = NSVG_PAINT_COLOR;
				}

				nvgGlobalCompositeBlendFunc(args.vg, NVG_ONE_MINUS_DST_COLOR, NVG_ONE);
				svgDraw(args.vg, sw->svg->handle);
				drawHalo(args);
			}
		}
		Widget::drawLayer(args, layer);
	}
};