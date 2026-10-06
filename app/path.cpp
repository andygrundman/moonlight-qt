#include "path.h"

#include <QtDebug>
#include <QDir>
#include <QStandardPaths>
#include <QSettings>
#include <QCoreApplication>

#ifdef Q_OS_DARWIN
#include <pwd.h>
#include <unistd.h>
#endif

QString Path::s_CacheDir;
QString Path::s_LogDir;
QString Path::s_BoxArtCacheDir;
QString Path::s_QmlCacheDir;

QString Path::getLogDir()
{
    Q_ASSERT(!s_LogDir.isEmpty());
    return s_LogDir;
}

QString Path::getBoxArtCacheDir()
{
    Q_ASSERT(!s_BoxArtCacheDir.isEmpty());
    return s_BoxArtCacheDir;
}

QString Path::getQmlCacheDir()
{
    Q_ASSERT(!s_QmlCacheDir.isEmpty());
    return s_QmlCacheDir;
}

QByteArray Path::readDataFile(QString fileName)
{
    QFile dataFile(getDataFilePath(fileName));
    if (!dataFile.open(QIODevice::ReadOnly)) {
        return {};
    }
    return dataFile.readAll();
}

void Path::writeCacheFile(QString fileName, QByteArray data)
{
    QDir cacheDir(s_CacheDir);

    // Create the cache path if it does not exist
    if (!cacheDir.exists()) {
        cacheDir.mkpath(".");
    }

    QFile dataFile(cacheDir.absoluteFilePath(fileName));
    if (dataFile.open(QIODevice::WriteOnly)) {
        dataFile.write(data);
    }
}

void Path::deleteCacheFile(QString fileName)
{
    QFile dataFile(QDir(s_CacheDir).absoluteFilePath(fileName));
    dataFile.remove();
}

QFileInfo Path::getCacheFileInfo(QString fileName)
{
    return QFileInfo(QDir(s_CacheDir), fileName);
}

QString Path::getDataFilePath(QString fileName)
{
    QString candidatePath;

    // Check the cache location first (used by Path::writeDataFile())
    candidatePath = QDir(s_CacheDir).absoluteFilePath(fileName);
    if (QFile::exists(candidatePath)) {
        qInfo() << "Found" << fileName << "at" << candidatePath;
        return candidatePath;
    }

    // Check the current directory
    candidatePath = QDir(QDir::currentPath()).absoluteFilePath(fileName);
    if (QFile::exists(candidatePath)) {
        qInfo() << "Found" << fileName << "at" << candidatePath;
        return candidatePath;
    }

    // Now check the data directories (for Linux, in particular)
    candidatePath = QStandardPaths::locate(QStandardPaths::AppDataLocation, fileName);
    if (!candidatePath.isEmpty() && QFile::exists(candidatePath)) {
        qInfo() << "Found" << fileName << "at" << candidatePath;
        return candidatePath;
    }

    // Now try the directory of our app installation (for Windows, if current dir doesn't find it)
    candidatePath = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(fileName);
    if (QFile::exists(candidatePath)) {
        qInfo() << "Found" << fileName << "at" << candidatePath;
        return candidatePath;
    }

    // Return the QRC embedded copy
    candidatePath = ":/data/" + fileName;
    qInfo() << "Found" << fileName << "at" << candidatePath;
    return QString(candidatePath);
}

void Path::initialize(bool portable)
{
    if (portable) {
        s_LogDir = QDir::currentPath();
        s_BoxArtCacheDir = QDir::currentPath() + "/boxart";
        s_QmlCacheDir = QDir::currentPath() + "/qmlcache";

        // In order for the If-Modified-Since logic to work in MappingFetcher,
        // the cache directory must be different than the current directory.
        s_CacheDir = QDir::currentPath() + "/cache";
    }
    else {
        s_LogDir = QDir::tempPath();
#ifdef Q_OS_DARWIN
        // We're running in a sandbox so it's a bit more work to get the home directory
        const struct passwd* user = getpwuid(getuid());
        if (user != nullptr && user->pw_dir != nullptr && user->pw_dir[0] != '\0') {
            const QString logDir = QDir(QFile::decodeName(user->pw_dir)).filePath("Library/Logs/Moonlight");
            if (QDir().mkpath(logDir)) {
                s_LogDir = logDir;
            }
            else {
                qWarning() << "Unable to create log directory:" << logDir
                           << "- using temporary directory:" << s_LogDir;
            }
        }
        else {
            qWarning() << "Unable to resolve the user home directory - using temporary directory:" << s_LogDir;
        }
#endif
        s_CacheDir = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
        s_BoxArtCacheDir = QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/boxart";
        s_QmlCacheDir = QStandardPaths::writableLocation(QStandardPaths::CacheLocation) + "/qmlcache";
    }
}
