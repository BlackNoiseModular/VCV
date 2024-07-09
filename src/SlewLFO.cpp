#include "plugin.hpp"


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
		IN_LIGHT,
		OUT_LIGHT,
		LIGHTS_LEN
	};

	SlewLFO() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(CURVE_PARAM, 0.f, 1.f, 0.f, "");
		configParam(RISE_PARAM, 0.f, 1.f, 0.f, "");
		configParam(FALL_PARAM, 0.f, 1.f, 0.f, "");
		configParam(MODE_PARAM, 0.f, 1.f, 0.f, "");
		configParam(RATE_PARAM, 0.f, 1.f, 0.f, "");
		configParam(CAPACITOR_PARAM, 0.f, 1.f, 0.f, "");
		configInput(RISE_INPUT, "");
		configInput(FALL_INPUT, "");
		configInput(IN_INPUT, "");
		configOutput(OUT_OUTPUT, "");
	}

	void process(const ProcessArgs& args) override {
	}
};


struct SlewLFOWidget : ModuleWidget {
	SlewLFOWidget(SlewLFO* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/SlewLFO.svg")));

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.14, 27.257)), module, SlewLFO::CURVE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.14, 45.257)), module, SlewLFO::RISE_PARAM));
		addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(10.14, 63.247)), module, SlewLFO::FALL_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(5.133, 80.944)), module, SlewLFO::MODE_PARAM));
		addParam(createParamCentered<CKSS>(mm2px(Vec(15.197, 80.944)), module, SlewLFO::RATE_PARAM));
		addParam(createParamCentered<VCVButton>(mm2px(Vec(10.147, 103.259)), module, SlewLFO::CAPACITOR_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.072, 95.296)), module, SlewLFO::RISE_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(15.124, 95.296)), module, SlewLFO::FALL_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.072, 112.021)), module, SlewLFO::IN_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(15.124, 112.021)), module, SlewLFO::OUT_OUTPUT));

		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(3.403, 104.003)), module, SlewLFO::IN_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(16.897, 104.003)), module, SlewLFO::OUT_LIGHT));
	}
};


Model* modelSlewLFO = createModel<SlewLFO, SlewLFOWidget>("SlewLFO");