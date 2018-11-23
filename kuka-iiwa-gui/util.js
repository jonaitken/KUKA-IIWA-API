//
//
//
exports.escapeHTML=function(str)
{
    return str.replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#x27;')
	.replace(/`/g, '&#x60;');
}
