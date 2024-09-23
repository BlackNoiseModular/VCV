#include "plugin.hpp"
#include "ChowDSP.hpp"

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
	enum CapacitorModifier {
		CAP_NONE,
		CAP_SLOW,
		CAP_SLOOOOW
	};
	enum RateMode {
		SLOW,
		FAST
	};
	enum SlewLFOMode {
		LFO,
		SLEW
	};
	enum LFOState {
		RISING,
		FALLING
	};

	SlewLFO() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CURVE_PARAM, 0.f, 1.f, 1.f, "Curve");
		configParam(RISE_PARAM, 0.f, 1.f, 0.5f, "Rise");
		configParam(FALL_PARAM, 0.f, 1.f, 0.5f, "Fall");
		configSwitch(MODE_PARAM, 0.f, 1.f, 0.f, "Mode", {"LFO", "Slew"});
		configSwitch(RATE_PARAM, 0.f, 1.f, 0.f, "Rate", {"Slow", "Fast"});
		auto capacitorExpander = configSwitch(CAPACITOR_PARAM, CAP_NONE, CAP_SLOOOOW, CAP_NONE, "Capacitor Expander", {"None", "Slow (~10uF)", "Sloooow (~100uF)"});
		capacitorExpander->description = "The capacitor expander allows the user to add their own capacitor to modify the slew rate.";
		configInput(RISE_INPUT, "Rise CV");
		configInput(FALL_INPUT, "Fall CV");
		configInput(IN_INPUT, "In");
		configOutput(OUT_OUTPUT, "Out");

		// calculate up/downsampling rates
		onSampleRateChange();

		updateCounter.setDivision(128);
	}

	void onSampleRateChange() override {
		float sampleRate = APP->engine->getSampleRate();
		for (int c = 0; c < PORT_MAX_CHANNELS; c++) {
			oversampler[c].setOversamplingIndex(oversamplingIndex);
			oversampler[c].reset(sampleRate);
		}
	}

	void updateKnobSettingsForMode(SlewLFOMode mode) {
		getParamQuantity(RISE_PARAM)->defaultValue = mode == LFO ? 0.5f : 0.0f;
		getParamQuantity(FALL_PARAM)->defaultValue = mode == LFO ? 0.5f : 0.0f;
	}

	void onReset(const ResetEvent& e) override {
		Module::onReset(e);

		const SlewLFOMode mode = static_cast<SlewLFOMode>(params[MODE_PARAM].getValue());

		// rise/fall knobs default values depend on mode
		updateKnobSettingsForMode(mode);

		params[RISE_PARAM].setValue(getParamQuantity(RISE_PARAM)->defaultValue);
		params[FALL_PARAM].setValue(getParamQuantity(FALL_PARAM)->defaultValue);
	}

	inline double crossfade(double a, double b, double p) {
		return a + (b - a) * p;
	}

	// oversampling
	chowdsp::VariableOversampling<6, double> oversampler[PORT_MAX_CHANNELS]; 	// uses a 2*6=12th order Butterworth filter
	int oversamplingIndex = 2; 	// default is 2^oversamplingIndex == x4 oversampling
	bool removeDCAtAudioRates = true;
	dsp::ClockDivider updateCounter;

	double out[PORT_MAX_CHANNELS] = {};
	double phase[PORT_MAX_CHANNELS] = {};
	bool state[PORT_MAX_CHANNELS] = {}; // false = rise, true = fall

	std::pair<double, double> getMinMaxSlewRates(RateMode rate, CapacitorModifier capacitor) {

		double slowestTime, fastestTime;
		switch (capacitor) {
			default:
			case CAP_NONE:
				if (rate == SLOW) {
					// slow: min 8s to 10V, max 8ms to 10V
					slowestTime = 8.; 		// 0.0625 Hz
					fastestTime = 0.008; 	// 62.5 Hz
				}
				else {
					// fast: min 200ms to 10V, max 120us to 10V
					slowestTime = 200e-3;	// 2.5 Hz
					fastestTime = 200e-6; 	// 2500 Hz
				}
				break;
			case CAP_SLOW:
				slowestTime = 20 * 60; 	// 20 minutes
				fastestTime = 1.2;		// 1.2 seconds
				break;
			case CAP_SLOOOOW:
				slowestTime = 33 * 60 * 60; 	// 33 hours
				fastestTime = 2 * 60; 			// 2 minutes
				break;
		}

		// rates are in volts per second, with rate being effectively time to reach 10V
		return {10. / slowestTime, 10. / fastestTime};
	}

	void process(const ProcessArgs& args) override {

		// minimum and maximum slopes in volts per second
		const RateMode rate = static_cast<RateMode>(params[RATE_PARAM].getValue());
		const CapacitorModifier capacitor = static_cast<CapacitorModifier>(params[CAPACITOR_PARAM].getValue());
		const SlewLFOMode mode = static_cast<SlewLFOMode>(params[MODE_PARAM].getValue());

		const auto [slewMin, slewMax] = getMinMaxSlewRates(rate, capacitor);
		// Amount of extra slew per voltage difference
		const double shapeScale = 1 / 10.;
		const double shape = (1 - params[CURVE_PARAM].getValue()) * 0.998;

		const double param_rise = params[RISE_PARAM].getValue() * 10.;
		const double param_fall = params[FALL_PARAM].getValue() * 10.;

		// this is the number of active polyphony engines, defined by the input
		const int numPolyphonyEngines = std::max({1, inputs[IN_INPUT].getChannels(), inputs[RISE_INPUT].getChannels(), inputs[FALL_INPUT].getChannels()});
		outputs[OUT_OUTPUT].setChannels(numPolyphonyEngines);

		// oversampling parts
		const int oversamplingRatio = oversampler[0].getOversamplingRatio();
		const bool oversampleOutput = (rate == FAST) && (oversamplingRatio > 1);
		const int oversampleRatioMain = (rate == FAST) ? oversamplingRatio : 1;
		const double sampleTimeOversample = args.sampleTime / oversampleRatioMain;

		// only need to do rarely, but update rise/fall defaults based on mode
		if (updateCounter.process()) {
			updateKnobSettingsForMode(mode);
		}


		for (int c = 0; c < numPolyphonyEngines; c++) {

			double* outBuffer = oversampler[c].getOSBuffer();
			for (int i = 0; i < oversampleRatioMain; i++) {
				double remainder = processForChannel(c, mode, slewMin, slewMax, shapeScale, shape, param_rise, param_fall, sampleTimeOversample);

				if (remainder > 0) {
					processForChannel(c, mode, slewMin, slewMax, shapeScale, shape, param_rise, param_fall, remainder);
				}
				outBuffer[i] = out[c];
			}

			const double outDownsampled = oversampleOutput ? oversampler[c].downsample() : out[c];
			if (removeDCAtAudioRates && rate == FAST && mode == LFO) {
				outputs[OUT_OUTPUT].setVoltage(outDownsampled - 5.f, c);
			}
			else {
				outputs[OUT_OUTPUT].setVoltage(outDownsampled, c);
			}

		}

		if (inputs[IN_INPUT].isConnected() && mode == SLEW) {
			const float in = inputs[IN_INPUT].getVoltage();
			setRedGreenLED(IN_LIGHT, in, args.sampleTime);
		}
		else {
			setRedGreenLED(IN_LIGHT, 0., args.sampleTime);
		}
		setRedGreenLED(OUT_LIGHT, out[0], args.sampleTime);
	}

	void setRedGreenLED(int firstLightId, float value, float deltaTime) {
		lights[firstLightId + 0].setBrightnessSmooth(value < 0 ? -value / 10.f : 0.f, deltaTime); 	// red
		lights[firstLightId + 1].setBrightnessSmooth(value > 0 ? +value / 10.f : 0.f, deltaTime);	// green
		lights[firstLightId + 2].setBrightness(0.f);												// blue
	}

	double processForChannel(int c, SlewLFOMode mode, double slewMin, double slewMax, double shapeScale, double shape, double param_rise, double param_fall, double sampleTime) {

		double in;
		switch (mode) {
			case SLEW:	{
				in = inputs[IN_INPUT].getPolyVoltage(c);
				break;
			}
			case LFO: {
				in = state[c] ? 0. : 10.;
				break;
			}
		}

		double riseCV = 0.0, fallCV = 0.0;
		if (inputs[RISE_INPUT].isConnected()) {
			riseCV = clamp(inputs[RISE_INPUT].getPolyVoltage(c), -5., 10.f);
		}
		if (inputs[FALL_INPUT].isConnected()) {
			fallCV = clamp(inputs[FALL_INPUT].getPolyVoltage(c), -5., 10.f);
		}

		riseCV += param_rise;
		fallCV += param_fall;

		double delta = in - out[c];
		double rateCV = 0.0;
		if (delta > 0.0) {
			rateCV = riseCV;
		}
		else if (delta < 0.0) {
			rateCV = fallCV;
		}
		rateCV *= 0.1;

		double pm_one = (delta > 0) - (delta < 0);
		double slew = slewMax * std::pow(slewMin / slewMax, rateCV);

		double diff = slew * crossfade(pm_one, shapeScale * delta, shape) * sampleTime;

		out[c] += diff;

		if (mode == SLEW) {
			out[c] = (delta > 0 && (out[c] > in)) ? in : out[c];
			out[c] = (delta < 0 && (out[c] < in)) ? in : out[c];
		}
		else {
			if (out[c] >= 10.) {
				state[c] = FALLING;

				double remainder = (out[c] - in) / diff;
				out[c] = 10.;

				return remainder * sampleTime;
			}
			else if (out[c] <= 0.) {
				state[c] = RISING;

				double remainder = (out[c] - in) / diff;
				out[c] = 0.;

				return remainder * sampleTime;
			}

		}
		return 0.;
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "removeDCAtAudioRates", json_boolean(removeDCAtAudioRates));
		json_object_set_new(rootJ, "oversamplingIndex", json_integer(oversampler[0].getOversamplingIndex()));

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {

		json_t* oversamplingIndexJ = json_object_get(rootJ, "oversamplingIndex");
		if (oversamplingIndexJ) {
			oversamplingIndex = json_integer_value(oversamplingIndexJ);
			onSampleRateChange();
		}

		json_t* removeDCAtAudioRatesJ = json_object_get(rootJ, "removeDCAtAudioRates");
		if (removeDCAtAudioRatesJ) {
			removeDCAtAudioRates = json_boolean_value(removeDCAtAudioRatesJ);
		}
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
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/slew_cap_0.svg")));
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/slew_cap_1.svg")));
		addFrame(Svg::load(asset::plugin(pluginInstance, "res/components/slew_cap_2.svg")));
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


	void appendContextMenu(Menu* menu) override {
		SlewLFO* module = static_cast<SlewLFO*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());

		menu->addChild(createIndexSubmenuItem("Oversampling (fast mode only)",
		{"Off", "x2", "x4", "x8", "x16"},
		[ = ]() {
			return module->oversamplingIndex;
		},
		[ = ](int mode) {
			module->oversamplingIndex = mode;
			module->onSampleRateChange();
		}));

		menu->addChild(createBoolPtrMenuItem("Centre waveform at audio rates", "", &module->removeDCAtAudioRates));
	}
};


Model* modelSlewLFO = createModel<SlewLFO, SlewLFOWidget>("SlewLFO");