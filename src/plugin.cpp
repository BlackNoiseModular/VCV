#include "plugin.hpp"


Plugin* pluginInstance;


void init(Plugin* p) {
	pluginInstance = p;

	// Add modules here
	p->addModel(modelCosmos);
	p->addModel(modelGomaII);
	p->addModel(modelSlewLFO);
}
