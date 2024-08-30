#include "plugin.hpp"

using namespace simd;

struct SlewLFO : Module {
	enum ParamId {
		CURVE_PARAM,
		RISE_PARAM,
		FALL_PARAM,
		MODE_PARAM,
		RATE_PARAM,
		CAPACITOR_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		RISE_INPUT,
		FALL_INPUT,
		IN_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		ENUMS(IN_LIGHT, 3),
		ENUMS(OUT_LIGHT, 3),
		LIGHTS_LEN
	};

	SlewLFO() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CURVE_PARAM, 0.f, 1.f, 0.f, "Curve");
		configParam(RISE_PARAM, 0.f, 1.f, 0.f, "Rise");
		configParam(FALL_PARAM, 0.f, 1.f, 0.f, "Fall");
		configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"LFO", "Slew"});
		configSwitch(RATE_PARAM, 0.f, 1.f, 0.f, "Rate", {"Slow", "Fast"});
		configSwitch(CAPACITOR_PARAM, 0.f, 1.f, 0.f, "Capacitor", {"None", "1uF"});
		configInput(RISE_INPUT, "Rise CV");
		configInput(FALL_INPUT, "Fall CV");
		configInput(IN_INPUT, "In");
		configOutput(OUT_OUTPUT, "Out");
	}

	float_4 out[4] = {};
	float_4 phases[4] = {};
	float_4 states[4] = {}; 	// 0 rising, 1 falling

	void process(const ProcessArgs& args) override {

		float_4 in[4] = {};
		float_4 riseCV[4] = {};
		float_4 fallCV[4] = {};

		// this is the number of active polyphony engines, defined by the input
		const int numPolyphonyEngines = std::max({1, inputs[IN_INPUT].getChannels(), inputs[RISE_INPUT].getChannels(), inputs[FALL_INPUT].getChannels()});

		// minimum and maximum slopes in volts per second
		const float slewMin = (params[RATE_PARAM].getValue()) ? 10.f / 120e-3 : 10.f / 12; 		// slow: 12s to 10V, fast: 120ms to 10V
		const float slewMax = (params[RATE_PARAM].getValue()) ? 10.f / 120e-6 : 10.f / 12e-3; 	// slow: 12 ms to 10V, fast: 120us to 10V
		// Amount of extra slew per voltage difference
		const float shapeScale = 1 / 10.f;
		const float shape = (1 - params[CURVE_PARAM].getValue()) * 0.998;

		const float_4 param_rise = params[RISE_PARAM].getValue() * 10.f;
		const float_4 param_fall = params[FALL_PARAM].getValue() * 10.f;

		outputs[OUT_OUTPUT].setChannels(numPolyphonyEngines);

		for (int c = 0; c < numPolyphonyEngines; c += 4) {
			if (params[MODE_PARAM].getValue()) {
				in[c / 4] = inputs[IN_INPUT].getVoltageSimd<float_4>(c);
			}
			else {
				states[c / 4] = ifelse(out[c / 4] >= 10.f, float_4::mask(), states[c / 4]);
				states[c / 4] = ifelse(out[c / 4] <= 0.f, 0.f, states[c / 4]);
				in[c / 4] = ifelse(states[c / 4], 0.f, 10.f);
			}


			if (inputs[RISE_INPUT].isConnected()) {
				riseCV[c / 4] = inputs[RISE_INPUT].getPolyVoltageSimd<float_4>(c);
			}
			if (inputs[FALL_INPUT].isConnected()) {
				fallCV[c / 4] = inputs[FALL_INPUT].getPolyVoltageSimd<float_4>(c);
			}

			riseCV[c / 4] += param_rise;
			fallCV[c / 4] += param_fall;

			float_4 delta = in[c / 4] - out[c / 4];
			float_4 delta_gt_0 = delta > 0.f;
			float_4 delta_lt_0 = delta < 0.f;

			float_4 rateCV = {};
			rateCV = ifelse(delta_gt_0, riseCV[c / 4], 0.f);
			rateCV = ifelse(delta_lt_0, fallCV[c / 4], rateCV) * 0.1f;

			float_4 pm_one = simd::sgn(delta);
			float_4 slew = slewMax * simd::pow(slewMin / slewMax, rateCV);

			out[c / 4] += slew * simd::crossfade(pm_one, shapeScale * delta, shape) * args.sampleTime;
			out[c / 4] = ifelse(delta_gt_0 & (out[c / 4] > in[c / 4]), in[c / 4], out[c / 4]);
			out[c / 4] = ifelse(delta_lt_0 & (out[c / 4] < in[c / 4]), in[c / 4], out[c / 4]);

			outputs[OUT_OUTPUT].setVoltageSimd(out[c / 4], c);
		}

		if (inputs[IN_INPUT].isConnected()) {
			setRedGreenLED(IN_LIGHT, in[0][0], args.sampleTime);
		}
		setRedGreenLED(OUT_LIGHT, out[0][0], args.sampleTime);

	}

	void setRedGreenLED(int firstLightId, float value, float deltaTime) {
		lights[firstLightId + 0].setBrightnessSmooth(value < 0 ? -value / 10.f : 0.f, deltaTime); 	// red
		lights[firstLightId + 1].setBrightnessSmooth(value > 0 ? +value / 10.f : 0.f, deltaTime);	// green
		lights[firstLightId + 2].setBrightness(0.f);												// blue
	}
};

struct SlewInLed : BlackNoiseLed {
	SlewInLed() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/slew_in.svg")));
	}
};

struct SlewOutLed : BlackNoiseLed {
	SlewOutLed() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/slew_out.svg")));
	}
};

struct CapacitorPanel : SvgSwitch {
	CapacitorPanel() {
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/slew_cap.svg")));
	}
};

struct SlewLFOWidget : ModuleWidget {
	SlewLFOWidget(SlewLFO* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/SlewLFO.svg")));

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<HexnutKnobBlack>(mm2px(Vec(10.14, 27.257)), module, SlewLFO::CURVE_PARAM));
		addParam(createParamCentered<HexnutKnobBlack>(mm2px(Vec(10.14, 45.257)), module, SlewLFO::RISE_PARAM));
		addParam(createParamCentered<HexnutKnobBlack>(mm2px(Vec(10.14, 63.247)), module, SlewLFO::FALL_PARAM));
		addParam(createParamCentered<DoepferSwitch>(mm2px(Vec(5.133, 80.944)), module, SlewLFO::MODE_PARAM));
		addParam(createParamCentered<DoepferSwitch>(mm2px(Vec(15.197, 80.944)), module, SlewLFO::RATE_PARAM));
		addParam(createParamCentered<CapacitorPanel>(mm2px(Vec(10.147, 103.259)), module, SlewLFO::CAPACITOR_PARAM));

		addInput(createInputCentered<GoldPort>(mm2px(Vec(5.072, 95.296)), module, SlewLFO::RISE_INPUT));
		addInput(createInputCentered<GoldPort>(mm2px(Vec(15.124, 95.296)), module, SlewLFO::FALL_INPUT));
		addInput(createInputCentered<GoldPort>(mm2px(Vec(5.072, 112.021)), module, SlewLFO::IN_INPUT));

		addOutput(createOutputCentered<GoldPort>(mm2px(Vec(15.124, 112.021)), module, SlewLFO::OUT_OUTPUT));

		addChild(createLightCentered<SlewInLed>(mm2px(Vec(3.403, 104.123)), module, SlewLFO::IN_LIGHT));
		addChild(createLightCentered<SlewOutLed>(mm2px(Vec(16.897, 104.123)), module, SlewLFO::OUT_LIGHT));
	}
};


Model* modelSlewLFO = createModel<SlewLFO, SlewLFOWidget>("SlewLFO");