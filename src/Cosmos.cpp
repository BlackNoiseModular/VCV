#include "plugin.hpp"

using namespace simd;

struct Cosmos : Module {
	enum ParamId {
		PAD_X_PARAM,
		PAD_Y_PARAM,
		PARAMS_LEN
	};
	enum InputId {
		X_INPUT,
		Y_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		XOR_GATE_OUTPUT,
		XOR_TRIG_OUTPUT,
		TZ_CLIPPER_OUTPUT,
		OR_GATE_OUTPUT,
		AND_GATE_OUTPUT,
		MAX_OUTPUT,
		MIN_OUTPUT,
		SUM_OUTPUT,
		ORG_TRIG_OUTPUT,
		X_OUTPUT,
		Y_OUTPUT,
		AND_TRIG_OUTPUT,
		NOR_TRIG_OUTPUT,
		INV_X_OUTPUT,
		INV_Y_OUTPUT,
		NAND_TRIG_OUTPUT,
		DIFF_OUTPUT,
		INV_MAX_OUTPUT,
		INV_MIN_OUTPUT,
		NOR_GATE_OUTPUT,
		NAND_GATE_OUTPUT,
		INV_TZ_CLIPPER_OUTPUT,
		XNOR_GATE_OUTPUT,
		XNOR_TRIG_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		XOR_LIGHT,
		MAX_LIGHT,
		MIN_LIGHT,
		SUM_LIGHT,
		X_LIGHT,
		Y_LIGHT,
		INV_X_LIGHT,
		INV_Y_LIGHT,
		DIFF_LIGHT,
		INV_MAX_LIGHT,
		INV_MIN_LIGHT,
		XNOR_LIGHT,
		LIGHTS_LEN
	};

	Cosmos() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(PAD_X_PARAM, 0.f, 1.f, 0.f, "");
		configParam(PAD_Y_PARAM, 0.f, 1.f, 0.f, "");
		configInput(X_INPUT, "X");
		configInput(Y_INPUT, "Y");
		configOutput(XOR_GATE_OUTPUT, "XOR gate");
		configOutput(XOR_TRIG_OUTPUT, "XOR trigger");
		configOutput(TZ_CLIPPER_OUTPUT, "Through-zero clipper");
		configOutput(OR_GATE_OUTPUT, "OR gate");
		configOutput(AND_GATE_OUTPUT, "AND gate");
		configOutput(MAX_OUTPUT, "Maximum");
		configOutput(MIN_OUTPUT, "Minimum");
		configOutput(SUM_OUTPUT, "Sum");
		configOutput(ORG_TRIG_OUTPUT, "OR-gate trigger");
		configOutput(X_OUTPUT, "X");
		configOutput(Y_OUTPUT, "Y");
		configOutput(AND_TRIG_OUTPUT, "AND trigger");
		configOutput(NOR_TRIG_OUTPUT, "NOR trigger");
		configOutput(INV_X_OUTPUT, "X (inverted)");
		configOutput(INV_Y_OUTPUT, "Y (inverted)");
		configOutput(NAND_TRIG_OUTPUT, "NAND trigger");
		configOutput(DIFF_OUTPUT, "Difference");
		configOutput(INV_MAX_OUTPUT, "Maximum (inverted)");
		configOutput(INV_MIN_OUTPUT, "Minimum (inverted)");
		configOutput(NOR_GATE_OUTPUT, "NOR gate");
		configOutput(NAND_GATE_OUTPUT, "NAND gate");
		configOutput(INV_TZ_CLIPPER_OUTPUT, "Ternary clipper (inverted)");
		configOutput(XNOR_GATE_OUTPUT, "XNOR gate");
		configOutput(XNOR_TRIG_OUTPUT, "XNOR trigger");
	}

	void process(const ProcessArgs& args) override {

		const int numActivePolyphonyChannels = std::max({1, inputs[X_INPUT].getChannels(), inputs[Y_INPUT].getChannels()});

		for (int c = 0; c < numActivePolyphonyChannels; c += 4) {

			const float_4 x = inputs[X_INPUT].getPolyVoltage(c);
			const float_4 y = inputs[Y_INPUT].getPolyVoltage(c);


			outputs[X_OUTPUT].setVoltageSimd<float_4>(x, c);
			outputs[Y_OUTPUT].setVoltageSimd<float_4>(y, c);

			outputs[SUM_OUTPUT].setVoltageSimd<float_4>(x + y, c);
			outputs[DIFF_OUTPUT].setVoltageSimd<float_4>(x - y, c);

			outputs[MAX_OUTPUT].setVoltageSimd<float_4>(ifelse(x > y, x, y), c);
			outputs[MIN_OUTPUT].setVoltageSimd<float_4>(ifelse(x > y, y, x), c);

			outputs[INV_X_OUTPUT].setVoltageSimd<float_4>(-x, c);
			outputs[INV_Y_OUTPUT].setVoltageSimd<float_4>(-y, c);
		}

		outputs[X_OUTPUT].setChannels(numActivePolyphonyChannels);
		outputs[Y_OUTPUT].setChannels(numActivePolyphonyChannels);
	}
};


struct CosmosWidget : ModuleWidget {
	CosmosWidget(Cosmos* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/Cosmos.svg")));

		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<VCVButton>(mm2px(Vec(6.47, 64.318)), module, Cosmos::PAD_X_PARAM));
		addParam(createParamCentered<VCVButton>(mm2px(Vec(64.275, 64.318)), module, Cosmos::PAD_Y_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.67, 64.347)), module, Cosmos::X_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(52.962, 64.347)), module, Cosmos::Y_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.677, 14.23)), module, Cosmos::XOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.981, 14.22)), module, Cosmos::XOR_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 21.201)), module, Cosmos::TZ_CLIPPER_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.428, 26.725)), module, Cosmos::OR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.23, 26.725)), module, Cosmos::AND_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.67, 39.245)), module, Cosmos::MAX_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.986, 39.245)), module, Cosmos::MIN_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 46.26)), module, Cosmos::SUM_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.44, 51.775)), module, Cosmos::ORG_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(24.889, 51.775)), module, Cosmos::X_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(45.757, 51.775)), module, Cosmos::Y_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.206, 51.775)), module, Cosmos::AND_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.453, 76.816)), module, Cosmos::NOR_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(24.902, 76.816)), module, Cosmos::INV_X_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(45.769, 76.816)), module, Cosmos::INV_Y_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.218, 76.816)), module, Cosmos::NAND_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 82.331)), module, Cosmos::DIFF_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.672, 89.346)), module, Cosmos::INV_MAX_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.989, 89.346)), module, Cosmos::INV_MIN_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.428, 101.866)), module, Cosmos::NOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.23, 101.865)), module, Cosmos::NAND_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 107.39)), module, Cosmos::INV_TZ_CLIPPER_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.677, 114.371)), module, Cosmos::XNOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.981, 114.361)), module, Cosmos::XNOR_TRIG_OUTPUT));

		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(35.331, 29.793)), module, Cosmos::XOR_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(26.279, 39.23)), module, Cosmos::MAX_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(44.376, 39.23)), module, Cosmos::MIN_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(35.3, 54.892)), module, Cosmos::SUM_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(29.372, 59.528)), module, Cosmos::X_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(41.292, 59.528)), module, Cosmos::Y_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(29.372, 69.052)), module, Cosmos::INV_X_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(41.292, 69.052)), module, Cosmos::INV_Y_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(35.3, 73.688)), module, Cosmos::DIFF_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(26.279, 89.35)), module, Cosmos::INV_MAX_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(44.376, 89.35)), module, Cosmos::INV_MIN_LIGHT));
		addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(35.331, 98.787)), module, Cosmos::XNOR_LIGHT));
	}
};


Model* modelCosmos = createModel<Cosmos, CosmosWidget>("Cosmos");