//    SAPF - Sound As Pure Form
//    Copyright (C) 2019 James McCartney
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "SoundFiles.hpp"
#include <valarray>

extern char gSessionTime[256];


class SFReaderOutputChannel;

class SFReader : public Object
{
	ExtAudioFileRef mXAF;
	int64_t mFramesRemaining;
	SFReaderOutputChannel* mOutputs;
	int mNumChannels;
	AudioBufferList* mABL;
	bool mFinished = false;
	
public:
	
	SFReader(ExtAudioFileRef inXAF, int inNumChannels, int64_t inDuration);
	
	~SFReader();

	virtual const char* TypeName() const override { return "SFReader"; }

	P<List> createOutputs(Thread& th);
	
	bool pull(Thread& th);
	void fulfillOutputs(int blockSize);
	void produceOutputs(int shrinkBy);
};

class SFReaderOutputChannel : public Gen
{
	friend class SFReader;
	P<SFReader> mSFReader;
	SFReaderOutputChannel* mNextOutput = nullptr;
	Z* mDummy = nullptr;
	
public:	
	SFReaderOutputChannel(Thread& th, SFReader* inSFReader)
        : Gen(th, itemTypeZ, true), mSFReader(inSFReader)
	{
	}
	
	~SFReaderOutputChannel()
	{
		if (mDummy) free(mDummy);
	}
	
	virtual void norefs() override
	{
		mOut = nullptr; 
		mSFReader = nullptr;
	}
		
	virtual const char* TypeName() const override { return "SFReaderOutputChannel"; }
	
	virtual void pull(Thread& th) override
	{
		if (mSFReader->pull(th)) {
			end();
		}
	}
	
};

SFReader::SFReader(ExtAudioFileRef inXAF, int inNumChannels, int64_t inDuration)
	: mXAF(inXAF), mNumChannels(inNumChannels), mFramesRemaining(inDuration), mABL(nullptr)
{
	mABL = (AudioBufferList*)calloc(1, sizeof(AudioBufferList) + (mNumChannels - 1) * sizeof(AudioBuffer));
}

SFReader::~SFReader()
{
	ExtAudioFileDispose(mXAF); free(mABL);
	SFReaderOutputChannel* output = mOutputs;
	do {
		SFReaderOutputChannel* next = output->mNextOutput;
		delete output;
		output = next;
	} while (output);
}

void SFReader::fulfillOutputs(int blockSize)
{
	mABL->mNumberBuffers = mNumChannels;
	SFReaderOutputChannel* output = mOutputs;
	size_t bufSize = blockSize * sizeof(Z);
	for (int i = 0; output; ++i, output = output->mNextOutput){
		Z* out;
		if (output->mOut)
			out = output->mOut->fulfillz(blockSize);
		else {
			if (!output->mDummy)
				output->mDummy = (Z*)calloc(output->mBlockSize, sizeof(Z));

			out = output->mDummy;
		}
			
		mABL->mBuffers[i].mNumberChannels = 1;
		mABL->mBuffers[i].mData = out;
		mABL->mBuffers[i].mDataByteSize = (UInt32)bufSize;
		memset(out, 0, bufSize);
	};
}

void SFReader::produceOutputs(int shrinkBy)
{
	SFReaderOutputChannel* output = mOutputs;
	do {
		if (output->mOut)
			output->produce(shrinkBy);
		output = output->mNextOutput;
	} while (output);
}

P<List> SFReader::createOutputs(Thread& th)
{
	P<List> s = new List(itemTypeV, mNumChannels);
	
	// fill s->mArray with ola's output channels.
    SFReaderOutputChannel* last = nullptr;
	P<Array> a = s->mArray;
	for (int i = 0; i < mNumChannels; ++i) {
        SFReaderOutputChannel* c = new SFReaderOutputChannel(th, this);
        if (last) last->mNextOutput = c;
        else mOutputs = c;
        last = c;
		a->add(new List(c));
	}
	
	return s;
}

bool SFReader::pull(Thread& th)
{
	if (mFramesRemaining == 0) 
		mFinished = true;

	if (mFinished) 
		return true;
	
	SFReaderOutputChannel* output = mOutputs;
	int blockSize = output->mBlockSize;
	if (mFramesRemaining > 0)
		blockSize = (int)std::min(mFramesRemaining, (int64_t)blockSize);
	
	fulfillOutputs(blockSize);
	
	// read file here.
	UInt32 framesRead = blockSize;
	OSStatus err = ExtAudioFileRead(mXAF, &framesRead, mABL);
	
	if (err || framesRead == 0) {
		mFinished = true;
	}
	
	produceOutputs(blockSize - framesRead);
	if (mFramesRemaining > 0) mFramesRemaining -= blockSize;
	
	return mFinished; 
}

void sfread(Thread& th, Arg filename, int64_t offset, int64_t frames)
{
	const char* path = ((String*)filename.o())->s;

	CFStringRef cfpath = CFStringCreateWithFileSystemRepresentation(0, path);
	if (!cfpath) {
		post("failed to create path\n");
		return;
	}
	CFReleaser cfpathReleaser(cfpath);
	
	CFURLRef url = CFURLCreateWithFileSystemPath(0, cfpath, kCFURLPOSIXPathStyle, false);
	if (!url) {
		post("failed to create url\n");
		return;
	}
	CFReleaser urlReleaser(url);
	
	ExtAudioFileRef xaf;
	OSStatus err = ExtAudioFileOpenURL(url, &xaf);

	cfpathReleaser.release();
	urlReleaser.release();
	
	if (err) {
		post("failed to open file %d\n", (int)err);
		return;
	}

	AudioStreamBasicDescription fileFormat;
	
	UInt32 propSize = sizeof(fileFormat);
	err = ExtAudioFileGetProperty(xaf, kExtAudioFileProperty_FileDataFormat, &propSize, &fileFormat);
	
	int numChannels = fileFormat.mChannelsPerFrame;

	AudioStreamBasicDescription clientFormat = {
		th.rate.sampleRate,
		kAudioFormatLinearPCM,
		kAudioFormatFlagsNativeFloatPacked | kAudioFormatFlagIsNonInterleaved,
		static_cast<UInt32>(sizeof(double)),
		1,
		static_cast<UInt32>(sizeof(double)),
		static_cast<UInt32>(numChannels),
		64,
		0
	};
	
	err = ExtAudioFileSetProperty(xaf, kExtAudioFileProperty_ClientDataFormat, sizeof(clientFormat), &clientFormat);
	if (err) {
		post("failed to set client data format\n");
		ExtAudioFileDispose(xaf);
		return;
	}
	
	err = ExtAudioFileSeek(xaf, offset);
	if (err) {
		post("seek failed %d\n", (int)err);
		ExtAudioFileDispose(xaf);
		return;
	}
	
	SFReader* sfr = new SFReader(xaf, numChannels, -1);
	
	th.push(sfr->createOutputs(th));
}

ExtAudioFileRef sfcreate(Thread& th, const char* path, int numChannels, double fileSampleRate, bool interleaved)
{
	if (fileSampleRate == 0.)
		fileSampleRate = th.rate.sampleRate;

	CFStringRef cfpath = CFStringCreateWithFileSystemRepresentation(0, path);
	if (!cfpath) {
		post("failed to create path '%s'\n", path);
		return nullptr;
	}
	CFReleaser cfpathReleaser(cfpath);
	
	CFURLRef url = CFURLCreateWithFileSystemPath(0, cfpath, kCFURLPOSIXPathStyle, false);
	if (!url) {
		post("failed to create url\n");
		return nullptr;
	}
	CFReleaser urlReleaser(url);
	
	AudioStreamBasicDescription fileFormat = {
		fileSampleRate,
		kAudioFormatLinearPCM,
		kAudioFormatFlagsNativeFloatPacked,
		static_cast<UInt32>(sizeof(float) * numChannels),
		1,
		static_cast<UInt32>(sizeof(float) * numChannels),
		static_cast<UInt32>(numChannels),
		32,
		0
	};
	
	int interleavedChannels = interleaved ? numChannels : 1;
	UInt32 interleavedBit = interleaved ? 0 : kAudioFormatFlagIsNonInterleaved;
	
	AudioStreamBasicDescription clientFormat = {
		th.rate.sampleRate,
		kAudioFormatLinearPCM,
		kAudioFormatFlagsNativeFloatPacked | interleavedBit,
		static_cast<UInt32>(sizeof(float) * interleavedChannels),
		1,
		static_cast<UInt32>(sizeof(float) * interleavedChannels),
		static_cast<UInt32>(numChannels),
		32,
		0
	};
		
	ExtAudioFileRef xaf;
	OSStatus err = ExtAudioFileCreateWithURL(url, kAudioFileWAVEType, &fileFormat, nullptr, kAudioFileFlags_EraseFile, &xaf);
	
	if (err) {
		post("failed to create file '%s'. err: %d\n", path, (int)err);
		return nullptr;
	}
	
	err = ExtAudioFileSetProperty(xaf, kExtAudioFileProperty_ClientDataFormat, sizeof(clientFormat), &clientFormat);
	if (err) {
		post("failed to set client data format\n");
		ExtAudioFileDispose(xaf);
		return nullptr;
	}
	
	return xaf;
}

std::atomic<int32_t> gFileCount = 0;

void makeRecordingPath(Arg filename, char* path, int len)
{
	if (filename.isString()) {
		const char* recDir = getenv("SAPF_RECORDINGS");
		if (!recDir || strlen(recDir)==0) recDir = "/tmp";
		snprintf(path, len, "%s/%s.wav", recDir, ((String*)filename.o())->s);
	} else {
		int32_t count = ++gFileCount;
		snprintf(path, len, "/tmp/sapf-%s-%04d.wav", gSessionTime, count);
	}
}

void sfwrite(Thread& th, V& v, Arg filename, bool openIt)
{
	std::vector<ZIn> in;
	
	int numChannels = 0;
		
	if (v.isZList()) {
		if (!v.isFinite()) indefiniteOp(">sf : s - indefinite number of frames", "");
		numChannels = 1;
		in.push_back(ZIn(v));
	} else {
		if (!v.isFinite()) indefiniteOp(">sf : s - indefinite number of channels", "");
		P<List> s = (List*)v.o();
		s = s->pack(th);
		Array* a = s->mArray();
		numChannels = (int)a->size();

		if (numChannels > kMaxSFChannels)
			throw errOutOfRange;
		
		bool allIndefinite = true;
		for (int i = 0; i < numChannels; ++i) {
			V va = a->at(i);
			if (va.isFinite()) allIndefinite = false;
			in.push_back(ZIn(va));
			va.o = nullptr;
		}

		s = nullptr;
		a = nullptr;
		
		if (allIndefinite) indefiniteOp(">sf : s - all channels have indefinite number of frames", "");
	}
	v.o = nullptr;

	char path[1024];
	
	makeRecordingPath(filename, path, 1024);
	
	ExtAudioFileRef xaf = sfcreate(th, path, numChannels, 0., true);
	if (!xaf) return;
	
	std::valarray<float> buf(0., numChannels * kBufSize);
	AudioBufferList abl;
	abl.mNumberBuffers = 1;
	abl.mBuffers[0].mNumberChannels = numChannels;
	abl.mBuffers[0].mData = &buf[0];
	abl.mBuffers[0].mDataByteSize = kBufSize * sizeof(float);
	
	int64_t framesPulled = 0;
	int64_t framesWritten = 0;
	bool done = false;
	while (!done) {
		int minn = kBufSize;
		memset(&buf[0], 0, kBufSize * numChannels);
		for (int i = 0; i < numChannels; ++i) {
			int n = kBufSize;
			bool imdone = in[i].fill(th, n, &buf[0]+i, numChannels);
			framesPulled += n;
			if (imdone) done = true;
			minn = std::min(n, minn);
		}

		abl.mBuffers[0].mDataByteSize = minn * sizeof(float);
		OSStatus err = ExtAudioFileWrite(xaf, minn, &abl);
		if (err) {
			post("ExtAudioFileWrite failed %d\n", (int)err);
			break;
		}

		framesWritten += minn;
	}
	
	post("wrote file '%s'  %d channels  %g secs\n", path, numChannels, framesWritten * th.rate.invSampleRate);
	
	ExtAudioFileDispose(xaf);
	
	if (openIt) {
		char cmd[1100];
		snprintf(cmd, 1100, "open \"%s\"", path);
		system(cmd);
	}
}
