#include "plugin.hpp"
#include "ChowDSP.hpp"

using namespace simd;

struct Cosmos : Module {
	enum ParamId {
		PAD_X_PARAM,
		PAD_Y_PARAM,
		THRESHOLD_PARAM,
		PRESSURE_PARAM,
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
		XOR_OUTPUT,
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
		XNOR_OUTPUT,
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

	// for outputting triggers
	PulseGenerator_4 logicalOrPulseGenerator[4];
	PulseGenerator_4 logicalAndPulseGenerator[4];
	PulseGenerator_4 logicalXorPulseGenerator[4];
	PulseGenerator_4 logicalNorPulseGenerator[4];
	PulseGenerator_4 logicalNandPulseGenerator[4];
	PulseGenerator_4 logicalXnorPulseGenerator[4];

	dsp::TSchmittTrigger<float_4> logicalOrGate[4];
	dsp::TSchmittTrigger<float_4> logicalAndGate[4];
	dsp::TSchmittTrigger<float_4> logicalXorGate[4];
	dsp::TSchmittTrigger<float_4> logicalNorGate[4];
	dsp::TSchmittTrigger<float_4> logicalNandGate[4];
	dsp::TSchmittTrigger<float_4> logicalXnorGate[4];

	dsp::BooleanTrigger xButtonTrigger;
	dsp::BooleanTrigger yButtonTrigger;

	// oversampling
	chowdsp::VariableOversampling<6, float_4> oversampler[OUTPUTS_LEN][4]; 	// uses a 2*6=12th order Butterworth filter
	int oversamplingIndex = 2; 	// default is 2^oversamplingIndex == x4 oversampling
	bool oversampleLogicOutputs = true;
	bool oversampleLogicGateOutputs = false;
	bool oversampleLogicTriggerOutputs = false;

	ParamQuantity* thresholdTrimmerQuantity{};
	ParamQuantity* pressureMaxQuantity{};
	// pressure for two pads (X, Y)
	float pressure[2] = {};

	Cosmos() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configParam(PAD_X_PARAM, 0.f, 1.f, 0.f, "Pad X");
		configParam(PAD_Y_PARAM, 0.f, 1.f, 0.f, "Pad Y");

		thresholdTrimmerQuantity = configParam(THRESHOLD_PARAM, .0f, 2.f, 1.f, "Gate/trigger threshold", "V");
		pressureMaxQuantity = configParam(PRESSURE_PARAM, .0f, 5.f, 1.f, "Pad max presssure", "V");

		configInput(X_INPUT, "X");
		configInput(Y_INPUT, "Y");

		configOutput(XOR_GATE_OUTPUT, "XOR gate");
		configOutput(XOR_TRIG_OUTPUT, "XOR trigger");
		configOutput(XOR_OUTPUT, "Through-zero clipper");
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
		configOutput(XNOR_OUTPUT, "Ternary clipper (inverted)");
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
		xButtonTrigger.process(params[PAD_X_PARAM].getValue());
		yButtonTrigger.process(params[PAD_Y_PARAM].getValue());

		const float_4 xPad = params[PRESSURE_PARAM].getValue() * pressure[PAD_X_PARAM] * xButtonTrigger.isHigh();
		const float_4 yPad = params[PRESSURE_PARAM].getValue() * pressure[PAD_Y_PARAM] * yButtonTrigger.isHigh();

		const float_4 threshold = params[THRESHOLD_PARAM].getValue();
		const int oversamplingRatio = oversampler[OR_OUTPUT][0].getOversamplingRatio();

		// trigger oversampling neccessitates gate oversampling
		if (oversampleLogicTriggerOutputs) {
			oversampleLogicGateOutputs = true;
		}
		// gate oversampling neccessitates logic outputs oversampling
		if (oversampleLogicGateOutputs) {
			oversampleLogicOutputs = true;
		}
		// disable oversampling if ratio is 1 (off)
		if (oversamplingRatio == 1) {
			oversampleLogicOutputs = false;
			oversampleLogicGateOutputs = false;
			oversampleLogicTriggerOutputs = false;
		}

		// loop over polyphony channels in blocks of 4
		for (int c = 0; c < numActivePolyphonyChannels; c += 4) {
			// x, y are normalled to the pad inputs
			const float_4 x = inputs[X_INPUT].getNormalPolyVoltageSimd<float_4>(xPad, c);
			const float_4 y = inputs[Y_INPUT].getNormalPolyVoltageSimd<float_4>(yPad, c);

			// basic main outputs
			outputs[X_OUTPUT].setVoltageSimd<float_4>(x, c);
			outputs[Y_OUTPUT].setVoltageSimd<float_4>(y, c);
			outputs[SUM_OUTPUT].setVoltageSimd<float_4>(0.5 * (x + y), c);
			// basic inverse outputs
			outputs[INV_X_OUTPUT].setVoltageSimd<float_4>(-x, c);
			outputs[INV_Y_OUTPUT].setVoltageSimd<float_4>(-y, c);
			outputs[DIFF_OUTPUT].setVoltageSimd<float_4>(0.5 * (x - y), c);

			// upsampled input arrays will be stored in these
			float_4* xBuffer = oversampler[c][X_OUTPUT].getOSBuffer();
			float_4* yBuffer = oversampler[c][Y_OUTPUT].getOSBuffer();
			if (oversamplingRatio > 1) {
				oversampler[c][X_OUTPUT].upsample(x);
				oversampler[c][Y_OUTPUT].upsample(y);
			}
			else {
				xBuffer[0] = x;
				yBuffer[0] = y;
			}

			// main logic outputs
			float_4* orBuffer = oversampler[c][OR_OUTPUT].getOSBuffer();
			float_4* andBuffer = oversampler[c][AND_OUTPUT].getOSBuffer();
			float_4* xorBuffer = oversampler[c][XOR_OUTPUT].getOSBuffer();
			const int oversampleRatioMain = oversampleLogicOutputs ? oversamplingRatio : 1;
			for (int i = 0; i < oversampleRatioMain; i++) {
				const float_4 x_ = xBuffer[i];
				const float_4 y_ = yBuffer[i];

				orBuffer[i] = ifelse(x_ > y_, x_, y_);
				andBuffer[i] = ifelse(x_ > y_, y_, x_);
				const float_4 clip_x = ifelse(x > abs(y_), abs(y_), ifelse(x_ < -abs(y_), -abs(y_), x_));
				xorBuffer[i] = ifelse(y_ > 0, -clip_x, clip_x);
			}

			// calculate logic outputs regardless if their outputs are used (LEDs still need to work, and easier to leave always on)
			const float_4 analogueOr = oversampleLogicOutputs ? oversampler[c][OR_OUTPUT].downsample() : orBuffer[0];
			outputs[OR_OUTPUT].setVoltageSimd<float_4>(analogueOr, c);
			const float_4 analogueNor = -analogueOr;
			outputs[NOR_OUTPUT].setVoltageSimd<float_4>(analogueNor, c);

			const float_4 analogueAnd = oversampleLogicOutputs ? oversampler[c][AND_OUTPUT].downsample() : andBuffer[0];
			outputs[AND_OUTPUT].setVoltageSimd<float_4>(analogueAnd, c);
			const float_4 analogueNand = -analogueAnd;
			outputs[NAND_OUTPUT].setVoltageSimd<float_4>(analogueNand, c);

			const float_4 analogueXor = oversampleLogicOutputs ? oversampler[c][XOR_OUTPUT].downsample() : xorBuffer[0];
			outputs[XOR_OUTPUT].setVoltageSimd<float_4>(analogueXor, c);
			const float_4 analogueXnor = -analogueXor;
			outputs[XNOR_OUTPUT].setVoltageSimd<float_4>(analogueXnor, c);


			// gate logic outputs
			float_4* orGateBuffer = oversampler[c][OR_GATE_OUTPUT].getOSBuffer();
			float_4* andGateBuffer = oversampler[c][AND_GATE_OUTPUT].getOSBuffer();
			float_4* xorGateBuffer = oversampler[c][XOR_GATE_OUTPUT].getOSBuffer();
			const int oversampleRatioGates = oversampleLogicGateOutputs ? oversamplingRatio : 1;
			for (int i = 0; i < oversampleRatioGates; i++) {
				orGateBuffer[i] = ifelse(orBuffer[i] > threshold, 10.f, 0.f);
				andGateBuffer[i] = ifelse(andBuffer[i] > threshold, 10.f, 0.f);
				// xor gate is a little different, are x and y close to within a tolerance
				xorGateBuffer[i] = ifelse(abs(xBuffer[i] - yBuffer[i]) > threshold, 10.f, 0.f);
			}

			// only bother with downsampling if there's an active output
			if (outputs[OR_GATE_OUTPUT].isConnected() || outputs[NOR_GATE_OUTPUT].isConnected()) {
				const float_4 orGateOut = oversampleLogicGateOutputs ? oversampler[c][OR_GATE_OUTPUT].downsample() : orGateBuffer[0];
				outputs[OR_GATE_OUTPUT].setVoltageSimd<float_4>(orGateOut, c);
				const float_4 norGateOut = 10.f - orGateOut;
				outputs[NOR_GATE_OUTPUT].setVoltageSimd<float_4>(norGateOut, c);
			}
			if (outputs[AND_GATE_OUTPUT].isConnected() || outputs[NAND_GATE_OUTPUT].isConnected()) {
				const float_4 andGateOut = oversampleLogicGateOutputs ? oversampler[c][AND_GATE_OUTPUT].downsample() : andGateBuffer[0];
				outputs[AND_GATE_OUTPUT].setVoltageSimd<float_4>(andGateOut, c);
				const float_4 nandGateOut = 10.f - andGateOut;
				outputs[NAND_GATE_OUTPUT].setVoltageSimd<float_4>(nandGateOut, c);
			}
			if (outputs[XOR_GATE_OUTPUT].isConnected() || outputs[XNOR_GATE_OUTPUT].isConnected()) {
				const float_4 xorGateOut = oversampleLogicGateOutputs ? oversampler[c][XOR_GATE_OUTPUT].downsample() : xorGateBuffer[0];
				outputs[XOR_GATE_OUTPUT].setVoltageSimd<float_4>(xorGateOut, c);
				const float_4 xnorGateOut = 10.f - xorGateOut;
				outputs[XNOR_GATE_OUTPUT].setVoltageSimd<float_4>(xnorGateOut, c);
			}


			// trigger outputs (derived from gates)
			float_4* orTriggerBuffer = oversampler[c][OR_TRIG_OUTPUT].getOSBuffer();
			float_4* norTriggerBuffer = oversampler[c][NOR_TRIG_OUTPUT].getOSBuffer();
			float_4* andTriggerBuffer = oversampler[c][AND_TRIG_OUTPUT].getOSBuffer();
			float_4* nandTriggerBuffer = oversampler[c][NAND_TRIG_OUTPUT].getOSBuffer();
			float_4* xorTriggerBuffer = oversampler[c][XOR_TRIG_OUTPUT].getOSBuffer();
			float_4* xnorTriggerBuffer = oversampler[c][XNOR_TRIG_OUTPUT].getOSBuffer();
			const int oversampleRatioTriggers = oversampleLogicTriggerOutputs ? oversamplingRatio : 1;
			const float deltaTime = args.sampleTime / oversampleRatioTriggers;

			for (int i = 0; i < oversampleRatioTriggers; i++) {

				const float_4 orTriggerHigh = logicalOrGate[c].process(orGateBuffer[i]);
				logicalOrPulseGenerator[c].trigger(orTriggerHigh, 1e-3);
				orTriggerBuffer[i] = ifelse(logicalOrPulseGenerator[c].process(deltaTime), 10.f, 0.f);
				// gate is literal inverse
				const float_4 norTiggerHigh = logicalNorGate[c].process(10.f - orGateBuffer[i]);
				logicalNorPulseGenerator[c].trigger(norTiggerHigh, 1e-3);
				norTriggerBuffer[i] = ifelse(logicalNorPulseGenerator[c].process(deltaTime), 10.f, 0.f);

				const float_4 andTriggerHigh = logicalAndGate[c].process(andGateBuffer[i]);
				logicalAndPulseGenerator[c].trigger(andTriggerHigh, 1e-3);
				andTriggerBuffer[i] = ifelse(logicalAndPulseGenerator[c].process(deltaTime), 10.f, 0.f);
				// gate is literal inverse
				const float_4 nandTriggerHigh = logicalNandGate[c].process(10.f - andGateBuffer[i]);
				logicalNandPulseGenerator[c].trigger(nandTriggerHigh, 1e-3);
				nandTriggerBuffer[i] = ifelse(logicalNandPulseGenerator[c].process(deltaTime), 10.f, 0.f);

				const float_4 xorTriggerHigh = logicalXorGate[c].process(xorGateBuffer[i]);
				logicalXorPulseGenerator[c].trigger(xorTriggerHigh, 1e-3);
				xorTriggerBuffer[i] = ifelse(logicalXorPulseGenerator[c].process(deltaTime), 10.f, 0.f);
				// gate is literal inverse
				const float_4 xnorTriggerHigh = logicalXnorGate[c].process(10.f - xorGateBuffer[i]);
				logicalXnorPulseGenerator[c].trigger(xnorTriggerHigh, 1e-3);
				xnorTriggerBuffer[i] = ifelse(logicalXnorPulseGenerator[c].process(deltaTime), 10.f, 0.f);
			}

			// updates trigger outputs (if they are connected)
			updateTriggerOutput(OR_TRIG_OUTPUT, c, orTriggerBuffer);
			updateTriggerOutput(NOR_TRIG_OUTPUT, c, norTriggerBuffer);
			updateTriggerOutput(AND_TRIG_OUTPUT, c, andTriggerBuffer);
			updateTriggerOutput(NAND_TRIG_OUTPUT, c, nandTriggerBuffer);
			updateTriggerOutput(XOR_TRIG_OUTPUT, c, xorTriggerBuffer);
			updateTriggerOutput(XNOR_TRIG_OUTPUT, c, xnorTriggerBuffer);

		}	 // end of polyphony loop

		if (numActivePolyphonyChannels == 1) {
			setRedGreenLED(OR_LIGHT, outputs[OR_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(AND_LIGHT, outputs[AND_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(XOR_LIGHT, outputs[XOR_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(SUM_LIGHT, outputs[SUM_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(X_LIGHT, outputs[X_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(Y_LIGHT, outputs[Y_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(DIFF_LIGHT, outputs[DIFF_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(INV_X_LIGHT, outputs[INV_X_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(INV_Y_LIGHT, outputs[INV_Y_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(NOR_LIGHT, outputs[NOR_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(NAND_LIGHT, outputs[NAND_OUTPUT].getVoltage(), args.sampleTime);
			setRedGreenLED(XNOR_LIGHT, outputs[XNOR_OUTPUT].getVoltage(), args.sampleTime);
		}
		else {
			setPolyphonicLED(OR_LIGHT);
			setPolyphonicLED(AND_LIGHT);
			setPolyphonicLED(XOR_LIGHT);
			setPolyphonicLED(SUM_LIGHT);
			setPolyphonicLED(X_LIGHT);
			setPolyphonicLED(Y_LIGHT);
			setPolyphonicLED(DIFF_LIGHT);
			setPolyphonicLED(INV_X_LIGHT);
			setPolyphonicLED(INV_Y_LIGHT);
			setPolyphonicLED(NOR_LIGHT);
			setPolyphonicLED(NAND_LIGHT);
			setPolyphonicLED(XNOR_LIGHT);
		}

		for (int outputId = 0; outputId < OUTPUTS_LEN; outputId++) {
			outputs[outputId].setChannels(numActivePolyphonyChannels);
		}
	}

	// if the output is connected, downsample (if appropriate) and set the voltage
	void updateTriggerOutput(int outputId, int channel, float_4* buffer) {
		if (outputs[outputId].isConnected()) {
			// only oversample if needed
			const float_4 triggerOut = oversampleLogicTriggerOutputs ? oversampler[channel][outputId].downsample() : buffer[0];
			outputs[outputId].setVoltageSimd<float_4>(triggerOut, channel);
		}
	}

	void setRedGreenLED(int firstLightId, float value, float deltaTime) {
		lights[firstLightId + 0].setBrightnessSmooth(value < 0 ? -value / 10.f : 0.f, deltaTime); 	// red
		lights[firstLightId + 1].setBrightnessSmooth(value > 0 ? +value / 10.f : 0.f, deltaTime);	// green
		lights[firstLightId + 2].setBrightness(0.f);												// blue
	}

	void setPolyphonicLED(int firstLightId) {
		lights[firstLightId + 0].setBrightness(0.f); 	// red
		lights[firstLightId + 1].setBrightness(0.f);	// green
		lights[firstLightId + 2].setBrightness(1.f);	// blue
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

// for context menu
struct ThresholdTrimmerSlider : ui::Slider {
	explicit ThresholdTrimmerSlider(ParamQuantity* q_) {
		quantity = q_;
		this->box.size.x = 200.0f;
	}
};
// for context menu
struct PressureMaxSlider : ui::Slider {
	explicit PressureMaxSlider(ParamQuantity* q_) {
		quantity = q_;
		this->box.size.x = 200.0f;
	}
};

struct CosmosLed : TSvgLight<RedGreenBlueLight> {

	CosmosLed() {

	}

	void draw(const DrawArgs& args) override {}
	void drawLayer(const DrawArgs& args, int layer) override {
		if (layer == 1) {

			if (!sw->svg)
				return;

			if (module && !module->isBypassed()) {

				for (auto s = sw->svg->handle->shapes; s; s = s->next) {
					s->fill.color = ((int)(color.a * 255) << 24) + (((int)(color.b * 255)) << 16) + (((int)(color.g * 255)) << 8) + (int)(color.r * 255);
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

struct CosmosLedXor : CosmosLed {
	CosmosLedXor() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_xor.svg")));
	}
};

struct CosmosLedOr : CosmosLed {
	CosmosLedOr() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_or.svg")));
	}
};

struct CosmosLedAnd : CosmosLed {
	CosmosLedAnd() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_and.svg")));
	}
};

struct CosmosLedX : CosmosLed {
	CosmosLedX() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_x.svg")));
	}
};

struct CosmosLedY : CosmosLed {
	CosmosLedY() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_y.svg")));
	}
};

struct CosmosLedDiff : CosmosLed {
	CosmosLedDiff() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_diff.svg")));
	}
};

struct CosmosLedSum : CosmosLed {
	CosmosLedSum() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_sum.svg")));
	}
};

struct CosmosLedXInv : CosmosLed {
	CosmosLedXInv() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_x_inv.svg")));
	}
};

struct CosmosLedYInv : CosmosLed {
	CosmosLedYInv() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_y_inv.svg")));
	}
};

struct CosmosLedNor : CosmosLed {
	CosmosLedNor() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_nor.svg")));
	}
};

struct CosmosLedNand : CosmosLed {
	CosmosLedNand() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_nand.svg")));
	}
};

struct CosmosLedXnor : CosmosLed {
	CosmosLedXnor() {
		this->setSvg(Svg::load(asset::plugin(pluginInstance, "res/components/cosmos_led_xnor.svg")));
	}
};

struct CosmosPad : app::SvgSwitch {
	bool dragging = false;
	Vec dragPosition = Vec(0, 0);

	CosmosPad() {
		momentary = true;
		// TODO: update
		addFrame(Svg::load(asset::system("res/ComponentLibrary/PB61303.svg")));
	}

	void onButton(const ButtonEvent& e) override {
		// storing starting position of drag (for mouse tracking)
		dragPosition = e.pos;

		SvgSwitch::onButton(e);
	}

	void onDragMove(const event::DragMove& e) override {

		// update the position
		float zoom = getAbsoluteZoom();
		Vec newPosition = dragPosition.plus(e.mouseDelta.div(zoom));
		dragPosition.x = newPosition.x;
		dragPosition.y = newPosition.y;

		// find distance from the centre of the pad
		math::Vec c = box.size.div(2);
		float dist = newPosition.minus(c).norm();
		// and store (so pressure is 1 at centre and 0 at the edge)
		Cosmos* thisModule = static_cast<Cosmos*>(module);
		thisModule->pressure[this->paramId] = 1.f - clamp(dist / c.x, 0.f, 1.f);

		e.consume(this);
		SvgSwitch::onDragMove(e);
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

		addParam(createParamCentered<CosmosPad>(mm2px(Vec(6.47, 64.318)), module, Cosmos::PAD_X_PARAM));
		addParam(createParamCentered<CosmosPad>(mm2px(Vec(64.275, 64.318)), module, Cosmos::PAD_Y_PARAM));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(17.67, 64.347)), module, Cosmos::X_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(52.962, 64.347)), module, Cosmos::Y_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.677, 14.23)), module, Cosmos::XOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.981, 14.22)), module, Cosmos::XOR_TRIG_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 21.201)), module, Cosmos::XOR_OUTPUT));
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
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(35.329, 107.39)), module, Cosmos::XNOR_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(17.677, 114.371)), module, Cosmos::XNOR_GATE_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(52.981, 114.361)), module, Cosmos::XNOR_TRIG_OUTPUT));

		addChild(createLightCentered<CosmosLedXor>(mm2px(Vec(35.331, 29.793)), module, Cosmos::XOR_LIGHT));
		addChild(createLightCentered<CosmosLedOr>(mm2px(Vec(26.279, 39.23)), module, Cosmos::OR_LIGHT));
		addChild(createLightCentered<CosmosLedAnd>(mm2px(Vec(44.376, 39.23)), module, Cosmos::AND_LIGHT));
		addChild(createLightCentered<CosmosLedSum>(mm2px(Vec(35.3, 54.892)), module, Cosmos::SUM_LIGHT));
		addChild(createLightCentered<CosmosLedX>(mm2px(Vec(29.372, 59.528)), module, Cosmos::X_LIGHT));
		addChild(createLightCentered<CosmosLedY>(mm2px(Vec(41.292, 59.528)), module, Cosmos::Y_LIGHT));
		addChild(createLightCentered<CosmosLedXInv>(mm2px(Vec(29.372, 69.052)), module, Cosmos::INV_X_LIGHT));
		addChild(createLightCentered<CosmosLedYInv>(mm2px(Vec(41.292, 69.052)), module, Cosmos::INV_Y_LIGHT));
		addChild(createLightCentered<CosmosLedDiff>(mm2px(Vec(35.3, 73.688)), module, Cosmos::DIFF_LIGHT));
		addChild(createLightCentered<CosmosLedNor>(mm2px(Vec(26.279, 89.35)), module, Cosmos::NOR_LIGHT));
		addChild(createLightCentered<CosmosLedNand>(mm2px(Vec(44.376, 89.35)), module, Cosmos::NAND_LIGHT));
		addChild(createLightCentered<CosmosLedXnor>(mm2px(Vec(35.331, 98.787)), module, Cosmos::XNOR_LIGHT));
	}

	void appendContextMenu(Menu* menu) override {
		Cosmos* module = static_cast<Cosmos*>(this->module);
		assert(module);

		menu->addChild(new MenuSeparator());


		menu->addChild(createSubmenuItem("Oversampling", "",
		[ = ](Menu * menu) {
			menu->addChild(createIndexSubmenuItem("Oversampling",
			{"Off", "x2", "x4", "x8"},
			[ = ]() {
				return module->oversamplingIndex;
			},
			[ = ](int mode) {
				module->oversamplingIndex = mode;
				module->onSampleRateChange();
			}));
			menu->addChild(createBoolPtrMenuItem("Oversample logic outputs", "", &module->oversampleLogicOutputs));
			menu->addChild(createBoolPtrMenuItem("Oversample logic gate outputs", "", &module->oversampleLogicGateOutputs));
			menu->addChild(createBoolPtrMenuItem("Oversample logic trigger outputs", "", &module->oversampleLogicTriggerOutputs));
		}));

		menu->addChild(new ThresholdTrimmerSlider(module->thresholdTrimmerQuantity));
		menu->addChild(new PressureMaxSlider(module->pressureMaxQuantity));

	}
};


Model* modelCosmos = createModel<Cosmos, CosmosWidget>("Cosmos");