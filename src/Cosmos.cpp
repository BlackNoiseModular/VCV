#include "plugin.hpp"
#include "ChowDSP.hpp"

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
		OR_OUTPUT,
		AND_OUTPUT,
		SUM_OUTPUT,
		OR_TRIG_OUTPUT,
		X_OUTPUT,
		Y_OUTPUT,
		AND_TRIG_OUTPUT,
		NOR_TRIG_OUTPUT,
		INV_X_OUTPUT,
		INV_Y_OUTPUT,
		NAND_TRIG_OUTPUT,
		DIFF_OUTPUT,
		NOR_OUTPUT,
		NAND_OUTPUT,
		NOR_GATE_OUTPUT,
		NAND_GATE_OUTPUT,
		INV_TZ_CLIPPER_OUTPUT,
		XNOR_GATE_OUTPUT,
		XNOR_TRIG_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		ENUMS(XOR_LIGHT, 3),
		ENUMS(OR_LIGHT, 3),
		ENUMS(AND_LIGHT, 3),
		ENUMS(SUM_LIGHT, 3),
		ENUMS(X_LIGHT, 3),
		ENUMS(Y_LIGHT, 3),
		ENUMS(INV_X_LIGHT, 3),
		ENUMS(INV_Y_LIGHT, 3),
		ENUMS(DIFF_LIGHT, 3),
		ENUMS(NOR_LIGHT, 3),
		ENUMS(NAND_LIGHT, 3),
		ENUMS(XNOR_LIGHT, 3),
		LIGHTS_LEN
	};


	// for boolean gates (unused for now)
	dsp::TSchmittTrigger<float_4> logicalOrSchmitt[4];
	dsp::TSchmittTrigger<float_4> logicalAndSchmitt[4];
	dsp::TSchmittTrigger<float_4> logicalXorSchmitt[4];
	// in theory we could use above for negated versions, but need falling edge detection too
	dsp::TSchmittTrigger<float_4> logicalNorSchmitt[4];
	dsp::TSchmittTrigger<float_4> logicalNandSchmitt[4];
	dsp::TSchmittTrigger<float_4> logicalXNorSchmitt[4];

	// for outputting triggers
	PulseGenerator_4 logicalOrPulseGenerator[4];
	PulseGenerator_4 logicalAndPulseGenerator[4];
	PulseGenerator_4 logicalXorPulseGenerator[4];
	PulseGenerator_4 logicalNorPulseGenerator[4];
	PulseGenerator_4 logicalNandPulseGenerator[4];
	PulseGenerator_4 logicalXnorPulseGenerator[4];

	BooleanTrigger_4 logicalOrGate[4];
	BooleanTrigger_4 logicalAndGate[4];
	BooleanTrigger_4 logicalXorGate[4];
	BooleanTrigger_4 logicalNorGate[4];
	BooleanTrigger_4 logicalNandGate[4];
	BooleanTrigger_4 logicalXnorGate[4];

	// oversampling
	chowdsp::VariableOversampling<6, float_4> oversampler[OUTPUTS_LEN][4]; 	// uses a 2*6=12th order Butterworth filter
	int oversamplingIndex = 2; 	// default is 2^oversamplingIndex == x4 oversampling
	bool oversampleLogicOutputs = true;
	bool oversampleLogicGateOutputs = false;
	bool oversampleLogicTriggerOutputs = false;


	Cosmos() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(PAD_X_PARAM, 0.f, 1.f, 0.f, "Pad X");
		configParam(PAD_Y_PARAM, 0.f, 1.f, 0.f, "Pad Y");

		configInput(X_INPUT, "X");
		configInput(Y_INPUT, "Y");

		configOutput(XOR_GATE_OUTPUT, "XOR gate");
		configOutput(XOR_TRIG_OUTPUT, "XOR trigger");
		configOutput(TZ_CLIPPER_OUTPUT, "Through-zero clipper");
		configOutput(OR_GATE_OUTPUT, "OR gate");
		configOutput(AND_GATE_OUTPUT, "AND gate");
		configOutput(OR_OUTPUT, "OR (maximum)");
		configOutput(AND_OUTPUT, "AND (minimum)");
		configOutput(SUM_OUTPUT, "Sum (mean)");
		configOutput(OR_TRIG_OUTPUT, "OR-gate trigger");
		configOutput(X_OUTPUT, "X");
		configOutput(Y_OUTPUT, "Y");
		configOutput(AND_TRIG_OUTPUT, "AND trigger");
		configOutput(NOR_TRIG_OUTPUT, "NOR trigger");
		configOutput(INV_X_OUTPUT, "X (inverted)");
		configOutput(INV_Y_OUTPUT, "Y (inverted)");
		configOutput(NAND_TRIG_OUTPUT, "NAND trigger");
		configOutput(DIFF_OUTPUT, "Difference");
		configOutput(NOR_OUTPUT, "NOR (maximum inverted)");
		configOutput(NAND_OUTPUT, "NAND (minimum inverted)");
		configOutput(NOR_GATE_OUTPUT, "NOR gate");
		configOutput(NAND_GATE_OUTPUT, "NAND gate");
		configOutput(INV_TZ_CLIPPER_OUTPUT, "Ternary clipper (inverted)");
		configOutput(XNOR_GATE_OUTPUT, "XNOR gate");
		configOutput(XNOR_TRIG_OUTPUT, "XNOR trigger");

		// calculate up/downsampling rates
		onSampleRateChange();
	}

	void onSampleRateChange() override {
		float sampleRate = APP->engine->getSampleRate();
		for (int c = 0; c < OUTPUTS_LEN; c++) {
			for (int i = 0; i < 4; i++) {
				oversampler[c][i].setOversamplingIndex(oversamplingIndex);
				oversampler[c][i].reset(sampleRate);
			}
		}
	}

	void process(const ProcessArgs& args) override {

		const int numActivePolyphonyChannels = std::max({1, inputs[X_INPUT].getChannels(), inputs[Y_INPUT].getChannels()});

		for (int c = 0; c < numActivePolyphonyChannels; c += 4) {

			const float_4 x = inputs[X_INPUT].getPolyVoltageSimd<float_4>(c);
			const float_4 y = inputs[Y_INPUT].getPolyVoltageSimd<float_4>(c);

			// main outputs
			outputs[X_OUTPUT].setVoltageSimd<float_4>(x, c);
			outputs[Y_OUTPUT].setVoltageSimd<float_4>(y, c);
			outputs[SUM_OUTPUT].setVoltageSimd<float_4>(0.5 * (x + y), c);

			const float_4 analogueOr = ifelse(x > y, x, y);
			const float_4 analogueAnd = ifelse(x > y, y, x);
			const float_4 clip_x = ifelse(x > abs(y), abs(y), ifelse(x < -abs(y), -abs(y), x));
			const float_4 analogueXor = ifelse(y > 0, -clip_x, clip_x);

			outputs[OR_OUTPUT].setVoltageSimd<float_4>(analogueOr, c);
			outputs[AND_OUTPUT].setVoltageSimd<float_4>(analogueAnd, c);
			outputs[TZ_CLIPPER_OUTPUT].setVoltageSimd<float_4>(analogueXor, c);

			// gate/trigger outputs
			{
				const float_4 orGateOut = ifelse(analogueOr > 0, 10.f, 0.f);
				const float_4 orTriggerHigh = logicalOrGate[c].process(analogueOr > 0);
				logicalOrPulseGenerator[c].trigger(orTriggerHigh, 1e-3);
				const float_4 orTriggerOut = ifelse(logicalOrPulseGenerator[c].process(args.sampleTime),  10.f, 0.f);
				outputs[OR_GATE_OUTPUT].setVoltageSimd<float_4>(orGateOut, c);
				outputs[OR_TRIG_OUTPUT].setVoltageSimd<float_4>(orTriggerOut, c);

				const float_4 andGateOut = ifelse(analogueAnd > 0, 10.f, 0.f);
				const float_4 andTriggerHigh = logicalAndGate[c].process(analogueAnd > 0);
				logicalAndPulseGenerator[c].trigger(andTriggerHigh, 1e-3);
				const float_4 andTriggerOut = ifelse(logicalAndPulseGenerator[c].process(args.sampleTime),  10.f, 0.f);
				outputs[AND_GATE_OUTPUT].setVoltageSimd<float_4>(andGateOut, c);
				outputs[AND_TRIG_OUTPUT].setVoltageSimd<float_4>(andTriggerOut, c);

				// xor gate is a little different
				const float_4 xorGateOut = ifelse((x > 0) ^ (y > 0), 10.f, 0.f);
				const float_4 xorTriggerHigh = logicalXorGate[c].process(xorGateOut > 0);
				logicalXorPulseGenerator[c].trigger(xorTriggerHigh, 1e-3);
				const float_4 xorTriggerOut = ifelse(logicalXorPulseGenerator[c].process(args.sampleTime),  10.f, 0.f);
				outputs[XOR_GATE_OUTPUT].setVoltageSimd<float_4>(xorGateOut, c);
				outputs[XOR_TRIG_OUTPUT].setVoltageSimd<float_4>(xorTriggerOut, c);
			}

			// inverse outputs
			outputs[INV_X_OUTPUT].setVoltageSimd<float_4>(-x, c);
			outputs[INV_Y_OUTPUT].setVoltageSimd<float_4>(-y, c);
			outputs[DIFF_OUTPUT].setVoltageSimd<float_4>(0.5 * (x - y), c);

			const float_4 analogueNor = -analogueOr;
			const float_4 analogueNand = -analogueAnd;
			const float_4 analogueXnor = -analogueXor;
			outputs[NOR_OUTPUT].setVoltageSimd<float_4>(analogueNor, c);
			outputs[NAND_OUTPUT].setVoltageSimd<float_4>(analogueNand, c);
			outputs[INV_TZ_CLIPPER_OUTPUT].setVoltageSimd<float_4>(analogueXnor, c);

			// inverse gate/trigger outputs
			{
				const float_4 norGateOut = ifelse(analogueNor < 0, 0.f, 10.f);
				const float_4 norTriggerHigh = logicalNorGate[c].process(analogueNor < 0);
				logicalNorPulseGenerator[c].trigger(norTriggerHigh, 1e-3);
				const float_4 norTriggerOut = ifelse(logicalNorPulseGenerator[c].process(args.sampleTime), 10.f, 0.f);
				outputs[NOR_GATE_OUTPUT].setVoltageSimd<float_4>(norGateOut, c);
				outputs[NOR_TRIG_OUTPUT].setVoltageSimd<float_4>(norTriggerOut, c);

				const float_4 nandGateOut = ifelse(analogueNand < 0, 0.f, 10.f);
				const float_4 nandTriggerHigh = logicalNandGate[c].process(analogueNand < 0);
				logicalNandPulseGenerator[c].trigger(nandTriggerHigh, 1e-3);
				const float_4 nandTriggerOut = ifelse(logicalNandPulseGenerator[c].process(args.sampleTime), 10.f, 0.f);
				outputs[NAND_GATE_OUTPUT].setVoltageSimd<float_4>(nandGateOut, c);
				outputs[NAND_TRIG_OUTPUT].setVoltageSimd<float_4>(nandTriggerOut, c);

				const float_4 xnorGateOut = ifelse((x < 0) ^ (y < 0), 10.f, 0.f);
				const float_4 xnorTriggerHigh = logicalXnorGate[c].process(xnorGateOut);
				logicalXnorPulseGenerator[c].trigger(xnorTriggerHigh, 1e-3);
				const float_4 xnorTriggerOut = ifelse(logicalXnorPulseGenerator[c].process(args.sampleTime), 10.f, 0.f);
				outputs[XNOR_GATE_OUTPUT].setVoltageSimd<float_4>(xnorGateOut, c);
				outputs[XNOR_TRIG_OUTPUT].setVoltageSimd<float_4>(xnorTriggerOut, c);

			}
		}

		if (numActivePolyphonyChannels == 1) {
			setRedGreenLED(OR_LIGHT, outputs[OR_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(AND_LIGHT, outputs[AND_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(XOR_LIGHT, outputs[TZ_CLIPPER_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(SUM_LIGHT, outputs[SUM_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(X_LIGHT, outputs[X_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(Y_LIGHT, outputs[Y_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(DIFF_LIGHT, outputs[DIFF_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(INV_X_LIGHT, outputs[INV_X_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(INV_Y_LIGHT, outputs[INV_Y_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(NOR_LIGHT, outputs[NOR_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(NAND_LIGHT, outputs[NAND_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(XNOR_LIGHT, outputs[INV_TZ_CLIPPER_OUTPUT].getVoltage(), args.sampleTime);
		}
		else {
			// TODO: handle polyphonic lights
		}

		for (int outputId = 0; outputId < OUTPUTS_LEN; outputId++) {
			outputs[outputId].setChannels(numActivePolyphonyChannels);
		}
	}

	void setRedGreenLED(int firstLightId, float value, float deltaTime) {
		lights[firstLightId + 0].setBrightnessSmooth(value < 0 ? -value / 10.f : 0.f, deltaTime); 	// red
		lights[firstLightId + 1].setBrightnessSmooth(value > 0 ? +value / 10.f : 0.f, deltaTime);	// green
		lights[firstLightId + 2].setBrightness(0.f);												// blue
	}

	json_t* dataToJson() override {
		json_t* rootJ = json_object();
		json_object_set_new(rootJ, "oversampleLogicOutputs", json_boolean(oversampleLogicOutputs));
		json_object_set_new(rootJ, "oversampleLogicGateOutputs", json_boolean(oversampleLogicGateOutputs));
		json_object_set_new(rootJ, "oversampleLogicTriggerOutputs", json_boolean(oversampleLogicTriggerOutputs));
		json_object_set_new(rootJ, "oversamplingIndex", json_integer(oversampler[0][0].getOversamplingIndex()));

		return rootJ;
	}

	void dataFromJson(json_t* rootJ) override {

		json_t* oversampleLogicOutputsJ = json_object_get(rootJ, "oversampleLogicOutputs");
		if (oversampleLogicOutputsJ) {
			oversampleLogicOutputs = json_boolean_value(oversampleLogicOutputsJ);
		}

		json_t* oversampleLogicGateOutputsJ = json_object_get(rootJ, "oversampleLogicGateOutputs");
		if (oversampleLogicGateOutputsJ) {
			oversampleLogicGateOutputs = json_boolean_value(oversampleLogicGateOutputsJ);
		}

		json_t* oversampleLogicTriggerOutputsJ = json_object_get(rootJ, "oversampleLogicTriggerOutputs");
		if (oversampleLogicTriggerOutputsJ) {
			oversampleLogicTriggerOutputs = json_boolean_value(oversampleLogicTriggerOutputsJ);
		}

		json_t* oversamplingIndexJ = json_object_get(rootJ, "oversamplingIndex");
		if (oversamplingIndexJ) {
			oversamplingIndex = json_integer_value(oversamplingIndexJ);
			onSampleRateChange();
		}
	}
};


struct CosmosWidget : ModuleWidget {
	CosmosWidget(Cosmos* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/panels/Cosmos.svg")));

		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewBlack>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewBlack>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(createParamCentered<VCVButton>(mm2px(Vec(6.47, 64.318)), module, Cosmos::PAD_X_PARAM));
		addParam(createParamCentered<VCVButton>(mm2px(Vec(64.275, 64.318)), module, Cosmos::PAD_Y_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.67, 64.347)), module, Cosmos::X_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(52.962, 64.347)), module, Cosmos::Y_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.677, 14.23)), module, Cosmos::XOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.981, 14.22)), module, Cosmos::XOR_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 21.201)), module, Cosmos::TZ_CLIPPER_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.428, 26.725)), module, Cosmos::OR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.23, 26.725)), module, Cosmos::AND_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.67, 39.245)), module, Cosmos::OR_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.986, 39.245)), module, Cosmos::AND_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 46.26)), module, Cosmos::SUM_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.44, 51.775)), module, Cosmos::OR_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(24.889, 51.775)), module, Cosmos::X_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(45.757, 51.775)), module, Cosmos::Y_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.206, 51.775)), module, Cosmos::AND_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.453, 76.816)), module, Cosmos::NOR_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(24.902, 76.816)), module, Cosmos::INV_X_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(45.769, 76.816)), module, Cosmos::INV_Y_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.218, 76.816)), module, Cosmos::NAND_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 82.331)), module, Cosmos::DIFF_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.672, 89.346)), module, Cosmos::NOR_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.989, 89.346)), module, Cosmos::NAND_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(10.428, 101.866)), module, Cosmos::NOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(60.23, 101.865)), module, Cosmos::NAND_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 107.39)), module, Cosmos::INV_TZ_CLIPPER_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.677, 114.371)), module, Cosmos::XNOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.981, 114.361)), module, Cosmos::XNOR_TRIG_OUTPUT));

		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(35.331, 29.793)), module, Cosmos::XOR_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(26.279, 39.23)), module, Cosmos::OR_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(44.376, 39.23)), module, Cosmos::AND_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(35.3, 54.892)), module, Cosmos::SUM_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(29.372, 59.528)), module, Cosmos::X_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(41.292, 59.528)), module, Cosmos::Y_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(29.372, 69.052)), module, Cosmos::INV_X_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(41.292, 69.052)), module, Cosmos::INV_Y_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(35.3, 73.688)), module, Cosmos::DIFF_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(26.279, 89.35)), module, Cosmos::NOR_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(44.376, 89.35)), module, Cosmos::NAND_LIGHT));
		addChild(createLightCentered<LargeLight<RedGreenBlueLight>>(mm2px(Vec(35.331, 98.787)), module, Cosmos::XNOR_LIGHT));
	}

	void appendContextMenu(Menu* menu) override {
		Cosmos* module = dynamic_cast<Cosmos*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());

		auto oversamplingRateMenu = createIndexSubmenuItem("Oversampling",
		{"Off", "x2", "x4", "x8"},
		[ = ]() {
			return module->oversamplingIndex;
		},
		[ = ](int mode) {
			module->oversamplingIndex = mode;
			module->onSampleRateChange();
		});

		menu->addChild(createSubmenuItem("Oversampling", "",
		[ = ](Menu * menu) {
			menu->addChild(oversamplingRateMenu);
			menu->addChild(createBoolPtrMenuItem("Oversample logic outputs", "", &module->oversampleLogicOutputs));
			menu->addChild(createBoolPtrMenuItem("Oversample logic gate outputs", "", &module->oversampleLogicGateOutputs));
			menu->addChild(createBoolPtrMenuItem("Oversample logic trigger outputs", "", &module->oversampleLogicTriggerOutputs));
		}));

	}
};


Model* modelCosmos = createModel<Cosmos, CosmosWidget>("Cosmos");