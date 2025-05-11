#pragma once

#if defined( FORGE_DLL ) || defined( FORGE_WITH_DLL )
	#ifdef FORGE_EXPORT_engine
		#define FORGE_ENGINE_API __declspec(dllexport)
		#define FORGE_ENGINE_API_TEMPLATE
	#else
		#define FORGE_ENGINE_API __declspec(dllimport)
		#define FORGE_ENGINE_API_TEMPLATE extern
	#endif
#else
	#define FORGE_ENGINE_API
	#define FORGE_ENGINE_API_TEMPLATE
#endif