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
		LIGHTS_LEN
	};

	dsp::ClockDivider updateCounter;

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

		updateCounter.setDivision(32);
	}

	std::vector<int> getPolyphonyStatus() {
		std::vector<int> result;
		for (int i = 0; i < 4; i++) {
			result.push_back(std::max(1, inputs[EXT_INPUT + i].getChannels()));
		}

		for (int i = 0; i < 4; i++) {
			if (outputs[EXT_OUTPUT + i].isConnected()){
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

		// get polyphonic status
		const std::vector<int> polyphonyStatus = getPolyphonyStatus();

		float_4 activeSum[4] = {};

		for (int m = 0; m < 4; m++) {
			
			const int numActiveChannels = polyphonyStatus[m];
			float gain = params[GAIN_EXT_PARAM + m].getValue();
			gain = params[MODE_EXT_PARAM + m].getValue() ? gain : 2 * gain - 1;

			for (int c = 0; c < numActiveChannels; c += 4) {
				
				activeSum[c/4] += inputs[EXT_INPUT + m].getNormalPolyVoltage(5.f, c) * gain;

				if (outputs[EXT_OUTPUT + m].isConnected()) {
					outputs[EXT_OUTPUT + m].setVoltageSimd<float_4>(activeSum[c/4], c);
					activeSum[c / 4] = 0.f;
				}
			}

			outputs[EXT_OUTPUT + m].setChannels(numActiveChannels);		
			
			if (numActiveChannels == 1) {
				setRedGreenLED(EXT_LIGHT + 3*m, inputs[EXT_INPUT + m].getVoltage(), args.sampleTime);
			}
		}
		
		// only need to do rarely, but update gain label based on whether we are in attenuator mode or attenuverter mode
		if (updateCounter.process())		
		{
			for (int m = 0; m < 4; m++) {
				getParamQuantity(GAIN_EXT_PARAM + m)->displayOffset = params[MODE_EXT_PARAM + m].getValue() ? 0.f : -100.f;
				getParamQuantity(GAIN_EXT_PARAM + m)->displayMultiplier = params[MODE_EXT_PARAM + m].getValue() ? 100.f : 200.f;
			}
		}
	}

	void setRedGreenLED(int firstLightId, float value, float deltaTime) {
		value = clamp(value / 10.f, -1.f, 1.f);
		lights[firstLightId + 0].setBrightnessSmooth(value < 0 ? -value : 0.f, deltaTime); 	// red
		lights[firstLightId + 1].setBrightnessSmooth(value > 0 ? +value : 0.f, deltaTime);	// green
		lights[firstLightId + 2].setBrightness(0.f);										// blue
	}
};


struct GomaIIWidget : ModuleWidget {
	GomaIIWidget(GomaII* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/GomaII.svg")));

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 14.344)), module, GomaII::GAIN_EXT_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(3.795, 15.55)), module, GomaII::MODE_EXT_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 41.844)), module, GomaII::GAIN_CH1_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(3.795, 43.05)), module, GomaII::MODE_CH1_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 69.344)), module, GomaII::GAIN_CH2_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(3.795, 70.55)), module, GomaII::MODE_CH2_PARAM));
		addParam(createParamCentered<Trimpot>(mm2px(Vec(13.146, 96.844)), module, GomaII::GAIN_CH3_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(3.795, 98.05)), module, GomaII::MODE_CH3_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.186, 30.55)), module, GomaII::EXT_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.186, 58.05)), module, GomaII::CH1_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.186, 85.55)), module, GomaII::CH2_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.186, 113.05)), module, GomaII::CH3_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.143, 30.55)), module, GomaII::EXT_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.143, 58.05)), module, GomaII::CH1_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.143, 85.55)), module, GomaII::CH2_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.143, 113.05)), module, GomaII::CH3_OUTPUT));

		addChild(createLightCentered<SmallLight<RedGreenBlueLight>>(mm2px(Vec(10.145, 24.552)), module, GomaII::EXT_LIGHT));
		addChild(createLightCentered<SmallLight<RedGreenBlueLight>>(mm2px(Vec(10.145, 52.05)), module, GomaII::CH1_LIGHT));
		addChild(createLightCentered<SmallLight<RedGreenBlueLight>>(mm2px(Vec(10.145, 79.55)), module, GomaII::CH2_LIGHT));
		addChild(createLightCentered<SmallLight<RedGreenBlueLight>>(mm2px(Vec(10.145, 107.05)), module, GomaII::CH3_LIGHT));
	}
};


Model* modelGomaII = createModel<GomaII, GomaIIWidget>("GomaII");