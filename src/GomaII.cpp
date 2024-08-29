#include "plugin.hpp"

using namespace simd;

struct GomaII : Module {
	enum ParamId {
		GAIN_EXT_PARAM,
		GAIN_CH1_PARAM,
		GAIN_CH2_PARAM,
		GAIN_CH3_PARAM,
		MODE_EXT_PARAM,
		MODE_CH1_PARAM,
		MODE_CH2_PARAM,
		MODE_CH3_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		EXT_INPUT,
		CH1_INPUT,
		CH2_INPUT,
		CH3_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		EXT_OUTPUT,
		CH1_OUTPUT,
		CH2_OUTPUT,
		CH3_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		ENUMS(EXT_LIGHT, 3),
		ENUMS(CH1_LIGHT, 3),
		ENUMS(CH2_LIGHT, 3),
		ENUMS(CH3_LIGHT, 3),
		EXPANDER_ACTIVE_LED,
		LIGHTS_LEN
	};
	enum NormalledVoltage {
		NORMALLED_5V,
		NORMALLED_10V
	};
	NormalledVoltage normalledVoltage = NORMALLED_5V;

	dsp::ClockDivider updateCounter;

	struct float_4_block {
		float_4 data[4] = {};
		int numActivePolyphonyChannels = 1;
	};
	float_4_block value[2];

	GomaII() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(GAIN_EXT_PARAM, 0.f, 1.f, 0.f, "Gain (External)", "%");
		configParam(GAIN_CH1_PARAM, 0.f, 1.f, 0.f, "Gain (Channel 1)", "%");
		configParam(GAIN_CH2_PARAM, 0.f, 1.f, 0.f, "Gain (Channel 2)", "%");
		configParam(GAIN_CH3_PARAM, 0.f, 1.f, 0.f, "Gain (Channel 3)", "%");

		configSwitch(MODE_EXT_PARAM, 0.f, 1.f, 1.f, "Mode (Channel Ext)", {"Attenuverter", "Attenuator"});
		configSwitch(MODE_CH1_PARAM, 0.f, 1.f, 1.f, "Mode (Channel 1)", {"Attenuverter", "Attenuator"});
		configSwitch(MODE_CH2_PARAM, 0.f, 1.f, 1.f, "Mode (Channel 2)", {"Attenuverter", "Attenuator"});
		configSwitch(MODE_CH3_PARAM, 0.f, 1.f, 1.f, "Mode (Channel 3)", {"Attenuverter", "Attenuator"});

		configInput(EXT_INPUT, "External");
		configInput(CH1_INPUT, "Channel 1");
		configInput(CH2_INPUT, "Channel 2");
		configInput(CH3_INPUT, "Channel 3");

		configOutput(EXT_OUTPUT, "External");
		configOutput(CH1_OUTPUT, "Channel 1");
		configOutput(CH2_OUTPUT, "Channel 2");
		configOutput(CH3_OUTPUT, "Channel 3");

		getLeftExpander().producerMessage = &value[0];
		getLeftExpander().consumerMessage = &value[1];

		updateCounter.setDivision(32);
	}

	std::vector<int> getPolyphonyStatus(int expanderPolyphonyChannels) {
		std::vector<int> result;
		for (int i = 0; i < 4; i++) {
			if (i == 0) {
				result.push_back(std::max({1, inputs[EXT_INPUT + i].getChannels(), expanderPolyphonyChannels}));
			}
			else {
				result.push_back(std::max(1, inputs[EXT_INPUT + i].getChannels()));
			}

		}

		for (int i = 0; i < 4; i++) {
			if (outputs[EXT_OUTPUT + i].isConnected()) {
				continue;
			}

			for (int j = i + 1; j < 4; j++) {
				result[j] = result[i] = std::max(result[i], result[j]);

				if (outputs[EXT_OUTPUT + j].isConnected()) {
					break;
				}
			}
		}

		return result;
	}

	void process(const ProcessArgs& args) override {


		// only need to do rarely, but update gain label based on whether we are in attenuator mode or attenuverter mode
		if (updateCounter.process()) {
			for (int m = 0; m < 4; m++) {
				getParamQuantity(GAIN_EXT_PARAM + m)->displayOffset = params[MODE_EXT_PARAM + m].getValue() ? 0.f : -100.f;
				getParamQuantity(GAIN_EXT_PARAM + m)->displayMultiplier = params[MODE_EXT_PARAM + m].getValue() ? 100.f : 200.f;
				getParamQuantity(GAIN_EXT_PARAM + m)->defaultValue = params[MODE_EXT_PARAM + m].getValue() ? 0.f : 0.5f;
			}
		}


		const float_4 normalledVoltageValue = (normalledVoltage == NORMALLED_5V) ? 5.f : 10.f;

		float_4 activeSum[4] = {};

		// if we have a left expander, it's output is normalled to Ext input (allows chained mixers)
		const float_4_block* leftExpanderData = (float_4_block*) getLeftExpander().consumerMessage;
		Module* leftModule = getLeftExpander().module;

		int numExpanderPolyphonyChannels = -1;
		if (leftModule && leftModule->getModel() == modelGomaII && leftExpanderData) {
			for (int c = 0; c < 4; c++) {
				activeSum[c] = leftExpanderData->data[c];
			}
			numExpanderPolyphonyChannels = leftExpanderData->numActivePolyphonyChannels;
		}
		const std::vector<int> polyphonyStatus = getPolyphonyStatus(numExpanderPolyphonyChannels);

		// loop over the four mixer channels (ext, ch1, ch2, ch3)
		for (int m = 0; m < 4; m++) {

			const int numActivePolyphonyChannels = polyphonyStatus[m];
			float gain = params[GAIN_EXT_PARAM + m].getValue();
			gain = params[MODE_EXT_PARAM + m].getValue() ? gain : 2 * gain - 1;

			// looper over polyphony channels
			for (int c = 0; c < numActivePolyphonyChannels; c += 4) {

				activeSum[c / 4] += inputs[EXT_INPUT + m].getNormalPolyVoltageSimd<float_4>(normalledVoltageValue, c) * gain;

				if (outputs[EXT_OUTPUT + m].isConnected()) {
					outputs[EXT_OUTPUT + m].setVoltageSimd<float_4>(activeSum[c / 4], c);
					activeSum[c / 4] = 0.f;
				}
			}

			outputs[EXT_OUTPUT + m].setChannels(numActivePolyphonyChannels);

			if (numActivePolyphonyChannels == 1) {
				setRedGreenLED(EXT_LIGHT + 3 * m, inputs[EXT_INPUT + m].getNormalVoltage(normalledVoltageValue[0]) * gain, args.sampleTime);
			}
		}

		Module* rightModule = getRightExpander().module;
		if (rightModule && rightModule->getModel() == modelGomaII) {
			// Get the producer message and cast to the correct pointer type.
			float_4_block* value = (float_4_block*) rightModule->getLeftExpander().producerMessage;

			// Write to the buffer
			for (int c = 0; c < 4; c++) {
				value->data[c] = activeSum[c];
			}
			value->numActivePolyphonyChannels = polyphonyStatus[3];

			// Request Rack's engine to flip the double-buffer upon the next engine frame.
			rightModule->getLeftExpander().requestMessageFlip();
		}

		// set LED to indicate expander active
		Module* leftExpanderModule = getLeftExpander().module;
		lights[EXPANDER_ACTIVE_LED].setBrightness((leftExpanderModule && leftExpanderModule->getModel() == modelGomaII));
	}

	void setRedGreenLED(int firstLightId, float value, float deltaTime) {
		value = clamp(value / 10.f, -1.f, 1.f);
		lights[firstLightId + 0].setBrightnessSmooth(value < 0 ? -value : 0.f, deltaTime); 	// red
		lights[firstLightId + 1].setBrightnessSmooth(value > 0 ? +value : 0.f, deltaTime);	// green
		lights[firstLightId + 2].setBrightness(0.f);										// blue
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "normalledVoltage", json_integer(normalledVoltage));

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {

		json_t* normalledVoltageJ = json_object_get(rootJ, "normalledVoltage");
		if (normalledVoltageJ) {
			normalledVoltage = (NormalledVoltage) json_integer_value(normalledVoltageJ);
		}
	}
};


struct GomaIIExtLed : SvgLight {
	static constexpr float backgroundGrey = 77.f / 255.f;

	GomaIIExtLed() {
		setSvg(APP->window->loadSvg(asset::plugin(pluginInstance, "res/components/goma_led_ext.svg")));
		this->addBaseColor(SCHEME_WHITE);
	}

	// this is needed otherwise the widget bugs out on zoom
	void draw(const DrawArgs& args) override {}

	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer == 1) {
			if (!sw->svg)
				return;

			if (module && !module->isBypassed()) {

				int fillColor = ((int)(color.a * 255) << 24) + (((int)(color.b * 255)) << 16) + (((int)(color.g * 255)) << 8) + (int)(color.r * 255);
				if (color.a == 0) {
					fillColor = ((int)(255) << 24) + (((int)(backgroundGrey * 255)) << 16) + (((int)(backgroundGrey * 255)) << 8) + (int)(backgroundGrey * 255);
				}

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

struct GomaCh1Led : BlackNoiseLed {
	GomaCh1Led() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/goma_1.svg")));
	}
};

struct GomaCh2Led : BlackNoiseLed {
	GomaCh2Led() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/goma_2.svg")));
	}
};

struct GomaCh3Led : BlackNoiseLed {
	GomaCh3Led() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/goma_3.svg")));
	}
};

struct GomaCh4Led : BlackNoiseLed {
	GomaCh4Led() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/goma_4.svg")));
	}
};

struct GomaIIWidget : ModuleWidget {
	GomaIIWidget(GomaII* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/GomaII.svg")));

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 14.344)), module, GomaII::GAIN_EXT_PARAM));
		addParam(createParamCentered<CKSSNarrow>(mm2px(Vec(3.795, 15.55)), module, GomaII::MODE_EXT_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 41.844)), module, GomaII::GAIN_CH1_PARAM));
		addParam(createParamCentered<CKSSNarrow>(mm2px(Vec(3.795, 43.05)), module, GomaII::MODE_CH1_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 69.344)), module, GomaII::GAIN_CH2_PARAM));
		addParam(createParamCentered<CKSSNarrow>(mm2px(Vec(3.795, 70.55)), module, GomaII::MODE_CH2_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 96.844)), module, GomaII::GAIN_CH3_PARAM));
		addParam(createParamCentered<CKSSNarrow>(mm2px(Vec(3.795, 98.05)), module, GomaII::MODE_CH3_PARAM));

		addInput(createInputCentered<GoldPort>(mm2px(Vec(5.186, 30.55)), module, GomaII::EXT_INPUT));
		addInput(createInputCentered<GoldPort>(mm2px(Vec(5.186, 58.05)), module, GomaII::CH1_INPUT));
		addInput(createInputCentered<GoldPort>(mm2px(Vec(5.186, 85.55)), module, GomaII::CH2_INPUT));
		addInput(createInputCentered<GoldPort>(mm2px(Vec(5.186, 113.05)), module, GomaII::CH3_INPUT));

		addOutput(createOutputCentered<GoldPort>(mm2px(Vec(15.143, 30.55)), module, GomaII::EXT_OUTPUT));
		addOutput(createOutputCentered<GoldPort>(mm2px(Vec(15.143, 58.05)), module, GomaII::CH1_OUTPUT));
		addOutput(createOutputCentered<GoldPort>(mm2px(Vec(15.143, 85.55)), module, GomaII::CH2_OUTPUT));
		addOutput(createOutputCentered<GoldPort>(mm2px(Vec(15.143, 113.05)), module, GomaII::CH3_OUTPUT));

		addChild(createLightCentered<GomaCh1Led>(mm2px(Vec(10.145, 24.552)), module, GomaII::EXT_LIGHT));
		addChild(createLightCentered<GomaCh2Led>(mm2px(Vec(10.145, 52.05)), module, GomaII::CH1_LIGHT));
		addChild(createLightCentered<GomaCh3Led>(mm2px(Vec(10.145, 79.55)), module, GomaII::CH2_LIGHT));
		addChild(createLightCentered<GomaCh4Led>(mm2px(Vec(10.145, 107.05)), module, GomaII::CH3_LIGHT));

		addChild(createLight<GomaIIExtLed>(mm2px(Vec(17.21, 24.657)), module, GomaII::EXPANDER_ACTIVE_LED));

	}

	void appendContextMenu(Menu* menu) override {
		GomaII* module = dynamic_cast<GomaII*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());
		menu->addChild(createIndexPtrSubmenuItem("Normalled input voltage", {"5V", "10V"}, &module->normalledVoltage));
	}
};


Model* modelGomaII = createModel<GomaII, GomaIIWidget>("GomaII");