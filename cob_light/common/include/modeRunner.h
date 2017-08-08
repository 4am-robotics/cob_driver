/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef MODERUNNER_H
#define MODERUNNER_H

class ModeRunner
{
public:
	ModeRunner();
	virtual ~ModeRunner();

	void start(Mode* mode);

	void stop();

	void join();

private:
	boost::shared_ptr<boost::thread> m_thread_ptr;

	boost::signals2::signal<void (color::rgba color)> m_sigFinished;

	void run();
};

#endif
