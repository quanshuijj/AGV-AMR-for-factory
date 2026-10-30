def nested_get(data, path, default=None):
    current = data
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return default
        current = current[key]
    return current


def get_topic(config, key, default=None):
    return nested_get(config, ["topics", key], default)


def get_frame(config, key, default=None):
    return nested_get(config, ["frames", key], default)


def get_alias_list(config, key):
    aliases = nested_get(config, ["topic_aliases", key], [])
    return aliases if isinstance(aliases, list) else []


def collect_record_topics(config):
    topics = []
    for _, topic_name in (config.get("topics") or {}).items():
        if topic_name:
            topics.append(topic_name)
    for _, topic_name in (config.get("optional_topics") or {}).items():
        if topic_name:
            topics.append(topic_name)
    deduped = []
    seen = set()
    for topic in topics:
        if topic not in seen:
            deduped.append(topic)
            seen.add(topic)
    return deduped
