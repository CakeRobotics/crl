def filter_by_type(obj, type_):
    return { key: data for key, data in obj.items() if data['type'] == type_ }
