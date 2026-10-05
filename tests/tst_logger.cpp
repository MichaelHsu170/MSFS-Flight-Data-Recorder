// Logger (logger.cpp): log rotation, header, level filtering, line format,
// and the C shim used by db.cpp.
#include "logger.h"
#include "logger_c.h"

#include <QFile>
#include <QTemporaryDir>
#include <QtTest>

class TstLogger : public QObject {
	Q_OBJECT

private:
	QTemporaryDir dir_;
	QString path_;

	QString logText() {
		QFile f(path_);
		if (!f.open(QIODevice::ReadOnly))
			return QString();
		return QString::fromUtf8(f.readAll());
	}

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		path_ = dir_.filePath("msfs_fdr_debug.log");
		QFile old(path_);
		QVERIFY(old.open(QIODevice::WriteOnly));
		old.write("previous run\n");
		old.close();
		Logger::init(Logger::Info, path_, "9.8.7");
	}

	void previousLogIsKeptAsOld() {
		QFile old(path_ + ".old");
		QVERIFY(old.open(QIODevice::ReadOnly));
		QCOMPARE(old.readAll(), QByteArray("previous run\n"));
	}

	void newLogStartsWithAHeader() {
		const QString text = logText();
		QVERIFY(text.startsWith("==== MSFS Flight Data Recorder v9.8.7 started (PID "));
		QVERIFY(!text.contains("previous run"));
	}

	void linesAreFormattedAndFilteredByLevel() {
		Logger::log(Logger::Warning, "Test", "warning line");
		Logger::log(Logger::Info, "Test", "info line");
		Logger::log(Logger::Trace, "Test", "trace line");
		Logger::log(Logger::Profile, "Test", "profile line");
		Logger::logf(Logger::Fatal, "Formatted", "%d-%s", 42, "x");
		const QString text = logText();
		QRegularExpression warn("\\d{4}-\\d{2}-\\d{2} \\d{2}:\\d{2}:\\d{2}\\.\\d{3} \\[WARN \\] \\[Test    \\] warning line\n");
		QVERIFY(warn.match(text).hasMatch());
		QVERIFY(text.contains("[INFO ] [Test    ] info line\n"));
		QVERIFY(text.contains("[FATAL] [Formatted] 42-x\n"));
		QVERIFY(!text.contains("trace line"));
		QVERIFY(!text.contains("profile line"));
	}

	void formattedMessagesAreCutTo1023Bytes() {
		const std::string longText(2000, 'y');
		Logger::logf(Logger::Info, "Long", "%s", longText.c_str());
		QVERIFY(logText().contains("[INFO ] [Long    ] " + QString(1023, 'y') + "\n"));
	}

	void cShimUsesTheSameLevels() {
		log_c(1, "DB", "c warning");
		log_cf(2, "DB", "c info %d", 7);
		log_cf(3, "DB", "c trace");
		const QString text = logText();
		QVERIFY(text.contains("[WARN ] [DB      ] c warning\n"));
		QVERIFY(text.contains("[INFO ] [DB      ] c info 7\n"));
		QVERIFY(!text.contains("c trace"));
	}

	void cShimDropsLevelsBeyondProfile() {
		log_cf(5, "DB", "beyond profile");
		QVERIFY(!logText().contains("beyond profile"));
	}

	void crashLoggingWritesToo() {
		Logger::logCrashf(Logger::Fatal, "Crash", "code %d", 5);
		QVERIFY(logText().contains("[FATAL] [Crash   ] code 5\n"));
	}

	void crashLoggingIsFilteredByLevelToo() {
		Logger::logCrashf(Logger::Trace, "Crash", "crash trace %d", 1);
		Logger::logCrash(Logger::Profile, "Crash", "crash profile");
		const QString text = logText();
		QVERIFY(!text.contains("crash trace"));
		QVERIFY(!text.contains("crash profile"));
	}

	// The file stays the first one, but the later call's level applies.
	void secondInitKeepsTheFileButTakesTheNewLevel() {
		Logger::init(Logger::Profile, dir_.filePath("other.log"));
		QVERIFY(!QFile::exists(dir_.filePath("other.log")));
		Logger::log(Logger::Info, "Test", "still here");
		Logger::log(Logger::Trace, "Test", "now traced");
		Logger::log(Logger::Profile, "Test", "now profiled");
		const QString text = logText();
		QVERIFY(text.contains("still here"));
		QVERIFY(text.contains("[TRACE] [Test    ] now traced\n"));
		QVERIFY(text.contains("[PROF ] [Test    ] now profiled\n"));
	}

	void outOfRangeLevelFromTheCShimIsTaggedUnknown() {
		log_c(-1, "DB", "odd level");
		QVERIFY(logText().contains("[?    ] [DB      ] odd level\n"));
	}

	void levelNames_data() {
		QTest::addColumn<QString>("name");
		QTest::addColumn<int>("level");
		QTest::newRow("fatal") << "FATAL" << (int)Logger::Fatal;
		QTest::newRow("warning lower") << "warning" << (int)Logger::Warning;
		QTest::newRow("info padded") << "  Info " << (int)Logger::Info;
		QTest::newRow("trace") << "TRACE" << (int)Logger::Trace;
		QTest::newRow("profile") << "PROFILE" << (int)Logger::Profile;
		QTest::newRow("unknown") << "LOUD" << (int)Logger::Info;
		QTest::newRow("empty") << "" << (int)Logger::Info;
	}

	void levelNames() {
		QFETCH(QString, name);
		QFETCH(int, level);
		QCOMPARE((int)Logger::levelFromString(name), level);
	}
};

QTEST_APPLESS_MAIN(TstLogger)
#include "tst_logger.moc"
