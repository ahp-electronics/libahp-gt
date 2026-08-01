/**
* \license
*    AHP common routines
*    Copyright (C) 2015-2023  Ilia Platone <info@iliaplatone.com>
*
*    This program is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef ROUTINES_H
#define ROUTINES_H

#ifdef _WIN32
#define DLL_EXPORT __declspec(dllexport)
#else
#define DLL_EXPORT extern
#endif

/** \defgroup Debug Debug features
* \{*/

#ifndef AHP_DEBUG
#define AHP_DEBUG
#define AHP_DEBUG_INFO 0
#define AHP_DEBUG_ERROR 1
#define AHP_DEBUG_WARNING 2
#define AHP_DEBUG_DEBUG 3
/**
* \brief set the debug level
* \param value the debug level
*/
DLL_EXPORT void ahp_set_debug_level(int32_t value);
/**
* \brief get the debug level
* \return The current debug level
*/
DLL_EXPORT int32_t ahp_get_debug_level();
/**
* \brief set the application name
* \param name the application name to be printed on logs
*/
DLL_EXPORT void ahp_set_app_name(const char* name);
/**
* \brief get the application name
* \return The current application name printed on logs
*/
DLL_EXPORT char* ahp_get_app_name();
/**
* \brief set the output log stream
* \param f The FILE stream pointer to set as standard output
*/
DLL_EXPORT void ahp_set_stdout(FILE *f);
/**
* \brief set the error log stream
* \param f The FILE stream pointer to set as standard error
*/
DLL_EXPORT void ahp_set_stderr(FILE *f);
#endif

/** \}*/
#endif // ROUTINES_H
